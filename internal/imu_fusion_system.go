package internal

import (
	"fmt"
	"math"
	"time"
)

// IMUFusionSystem is the main struct orchestrating the fusion pipeline.
type IMUFusionSystem struct {
	acq        *DataAcquisition
	sync       *Synchronizer
	calib      []*IMU
	cloud      *PointCloud
	cloud3D    *PointCloud3D     // 3D point cloud for 3D mode
	positions  []Point           // per-IMU 2D position state
	positions3D []Point3D        // per-IMU 3D position state
	velocities []Point           // per-IMU 2D velocity state
	velocities3D []Point3D       // per-IMU 3D velocity state
	orientations []Quaternion    // per-IMU orientation state (6DOF support)
	angularVelocities []Point3D  // per-IMU angular velocity state
	lastTime   time.Time         // last timestamp for integration
	noiseLevel float64           // IMU noise level for uncertainty calculation
	reference  []Point           // 2D reference geometry for rigid transform
	reference3D []Point3D        // 3D reference geometry for rigid transform
	imuCount   int               // number of IMUs
	is3D       bool              // whether to use 3D mode
}

// NewIMUFusionSystem initializes the IMU fusion system.
func NewIMUFusionSystem(imuCount int) (*IMUFusionSystem, error) {
	return NewIMUFusionSystemWithMode(imuCount, false) // Default to 2D mode for backward compatibility
}

// NewIMUFusionSystemWithMode initializes the IMU fusion system with specified dimensionality.
func NewIMUFusionSystemWithMode(imuCount int, is3D bool) (*IMUFusionSystem, error) {
	sync := NewSynchronizer()
	acq := NewDataAcquisition(imuCount, sync) // Pass synchronizer to acquisition
	calib := make([]*IMU, imuCount)
	for i := 0; i < imuCount; i++ {
		calib[i] = NewIMU()
		calib[i].ID = i // Assign ID
	}
	
	var cloud *PointCloud
	var cloud3D *PointCloud3D
	if is3D {
		cloud3D = NewPointCloud3D()
	} else {
		cloud = NewPointCloud()
	}
	
	positions := make([]Point, imuCount)
	positions3D := make([]Point3D, imuCount)
	velocities := make([]Point, imuCount)
	velocities3D := make([]Point3D, imuCount)
	orientations := make([]Quaternion, imuCount)
	angularVelocities := make([]Point3D, imuCount)
	
	// Initialize orientations to identity quaternions
	for i := 0; i < imuCount; i++ {
		orientations[i] = NewQuaternion()
	}
	
	now := time.Now()
	noise := 0.1 // default noise level
	
	// Reference geometry setup
	d := 1.0
	ref := []Point{{0, 0}, {d, 0}, {d, d}, {0, d}} // Assuming 4 IMUs for 2D ref geometry
	ref3D := []Point3D{{0, 0, 0}, {d, 0, 0}, {d, d, 0}, {0, d, 0}} // 4 IMUs in XY plane for 3D ref geometry
	
	if imuCount != 4 {
		// TODO: Handle reference geometry for different imu counts
		fmt.Println("Warning: Reference geometry is hardcoded for 4 IMUs.")
	}

	return &IMUFusionSystem{
		acq:               acq,
		sync:              sync,
		calib:             calib,
		cloud:             cloud,
		cloud3D:           cloud3D,
		positions:         positions,
		positions3D:       positions3D,
		velocities:        velocities,
		velocities3D:      velocities3D,
		orientations:      orientations,
		angularVelocities: angularVelocities,
		lastTime:          now,
		noiseLevel:        noise,
		reference:         ref,
		reference3D:       ref3D,
		imuCount:          imuCount,
		is3D:              is3D,
	}, nil
}

// Start starts the data acquisition and processing loop.
func (sys *IMUFusionSystem) Start() {
	sys.acq.Start()          // Start data collection goroutines
	go sys.processDataLoop() // Start processing loop
}

// Stop stops the data acquisition and processing.
func (sys *IMUFusionSystem) Stop() {
	sys.acq.Stop()
	// TODO: Potentially add a way to signal processDataLoop to stop gracefully
}

// processDataLoop runs the main fusion logic.
func (sys *IMUFusionSystem) processDataLoop() {
	for {
		// Get aligned data frames from the synchronizer
		alignedFrames := sys.sync.GetAlignedData(sys.imuCount)
		if len(alignedFrames) == 0 {
			time.Sleep(1 * time.Millisecond) // Wait if no aligned data
			continue
		}

		for _, frame := range alignedFrames {
			// Assuming frame is sorted by IMUID or has a known order
			// Use the timestamp from the first data point in the frame
			now := frame[0].Timestamp
			dt := now.Sub(sys.lastTime).Seconds()
			if dt <= 0 { // Avoid division by zero or negative time steps
				dt = 1e-9 // Use a very small positive dt
			}
			sys.lastTime = now

			if sys.is3D {
				sys.processFrame3D(frame, dt)
			} else {
				sys.processFrame2D(frame, dt)
			}
		}
	}
}

// processFrame2D processes a single data frame in 2D mode
func (sys *IMUFusionSystem) processFrame2D(frame []IMUData, dt float64) {
	currentPositions := make([]Point, sys.imuCount)
	// Integrate data for each IMU in the aligned frame
	for _, data := range frame {
		imuIndex := data.IMUID // Use IMUID to index into calibration/state arrays
		if imuIndex >= sys.imuCount {
			fmt.Printf("Error: IMUID %d out of bounds\n", imuIndex)
			continue // Skip data point if ID is invalid
		}

		// Calibrate acceleration
		ax, ay := sys.calib[imuIndex].ApplyCalibration(data.Acceleration[0], data.Acceleration[1])

		// Integrate velocity and position
		sys.velocities[imuIndex].X += ax * dt
		sys.velocities[imuIndex].Y += ay * dt
		sys.positions[imuIndex].X += sys.velocities[imuIndex].X * dt
		sys.positions[imuIndex].Y += sys.velocities[imuIndex].Y * dt

		currentPositions[imuIndex] = sys.positions[imuIndex]

		// Add to point cloud
		sys.cloud.AddPoint(sys.positions[imuIndex].X, sys.positions[imuIndex].Y)
	}

	// Estimate uncertainties per IMU
	uncertainties := make([]float64, sys.imuCount)
	for i := 0; i < sys.imuCount; i++ {
		u := NewUncertainty(sys.noiseLevel, dt)
		uncertainties[i] = u.Estimate()
	}

	// Geometric fusion
	posList := make([]Position, sys.imuCount)
	for i := 0; i < sys.imuCount; i++ {
		posList[i] = Position{X: currentPositions[i].X, Y: currentPositions[i].Y, R: uncertainties[i]}
	}
	_, fused := GeometricFusion2D(posList)

	// Rigid body transformation enforcement (using current positions and reference)
	aligned, _, _ := Procrustes(currentPositions, sys.reference)

	// FIXME: Apply the transformation derived from Procrustes (rotation, scale, translation)
	// to the 'fused' point. The current implementation below doesn't use the Procrustes result effectively.
	// It just averages the 'aligned' points which isn't the correct way to apply the constraint.
	var avgAlignedX, avgAlignedY float64
	for _, p := range aligned {
		avgAlignedX += p.X
		avgAlignedY += p.Y
	}
	// FIXME: Placeholder: Replace fused with average of aligned points for now
	// fused.X = avgAlignedX / float64(len(aligned))
	// fused.Y = avgAlignedY / float64(len(aligned))

	// Point cloud refinement
	neighbors := sys.cloud.RadiusSearch(fused.X, fused.Y, fused.R)
	sumX, sumY := 0.0, 0.0
	count := len(neighbors)
	for _, pt := range neighbors {
		sumX += pt.X
		sumY += pt.Y
	}
	finalX, finalY := fused.X, fused.Y
	if count > 0 {
		finalX = sumX / float64(count)
		finalY = sumY / float64(count)
	}

	// Output fused and refined position
	fmt.Printf("2D Fused position: (%.3f, %.3f)\n", finalX, finalY)
}

// processFrame3D processes a single data frame in 3D mode with 6DOF
func (sys *IMUFusionSystem) processFrame3D(frame []IMUData, dt float64) {
	currentPositions := make([]Point3D, sys.imuCount)
	// Integrate data for each IMU in the aligned frame
	for _, data := range frame {
		imuIndex := data.IMUID // Use IMUID to index into calibration/state arrays
		if imuIndex >= sys.imuCount {
			fmt.Printf("Error: IMUID %d out of bounds\n", imuIndex)
			continue // Skip data point if ID is invalid
		}

		// Calibrate acceleration and angular velocity
		ax, ay, az := sys.calib[imuIndex].ApplyCalibration3D(data.Acceleration[0], data.Acceleration[1], data.Acceleration[2])
		wx, wy, wz := sys.calib[imuIndex].ApplyCalibration3D(data.AngularVelocity[0], data.AngularVelocity[1], data.AngularVelocity[2])

		// Integrate linear velocity and position
		sys.velocities3D[imuIndex].X += ax * dt
		sys.velocities3D[imuIndex].Y += ay * dt
		sys.velocities3D[imuIndex].Z += az * dt
		sys.positions3D[imuIndex].X += sys.velocities3D[imuIndex].X * dt
		sys.positions3D[imuIndex].Y += sys.velocities3D[imuIndex].Y * dt
		sys.positions3D[imuIndex].Z += sys.velocities3D[imuIndex].Z * dt

		// Integrate angular velocity to update orientation (quaternion integration)
		// Store current angular velocity
		sys.angularVelocities[imuIndex] = Point3D{X: wx, Y: wy, Z: wz}
		
		// Simple quaternion integration: q(t+dt) = q(t) + 0.5 * dt * omega_q * q(t)
		// where omega_q is the quaternion representation of angular velocity
		omegaQ := Quaternion{W: 0, X: wx, Y: wy, Z: wz}
		qDot := omegaQ.Multiply(sys.orientations[imuIndex])
		qDot.W *= 0.5 * dt
		qDot.X *= 0.5 * dt
		qDot.Y *= 0.5 * dt
		qDot.Z *= 0.5 * dt
		
		sys.orientations[imuIndex].W += qDot.W
		sys.orientations[imuIndex].X += qDot.X
		sys.orientations[imuIndex].Y += qDot.Y
		sys.orientations[imuIndex].Z += qDot.Z
		sys.orientations[imuIndex].Normalize() // Keep quaternion normalized

		currentPositions[imuIndex] = sys.positions3D[imuIndex]

		// Add to 3D point cloud
		sys.cloud3D.AddPoint(sys.positions3D[imuIndex].X, sys.positions3D[imuIndex].Y, sys.positions3D[imuIndex].Z)
	}

	// Estimate uncertainties per IMU
	uncertainties := make([]float64, sys.imuCount)
	for i := 0; i < sys.imuCount; i++ {
		u := NewUncertainty(sys.noiseLevel, dt)
		uncertainties[i] = u.Estimate()
	}

	// 3D Geometric fusion
	posList := make([]Position3D, sys.imuCount)
	for i := 0; i < sys.imuCount; i++ {
		posList[i] = Position3D{X: currentPositions[i].X, Y: currentPositions[i].Y, Z: currentPositions[i].Z, R: uncertainties[i]}
	}
	_, fused := GeometricFusion3D(posList)

	// TODO: Implement 3D Procrustes analysis for rigid body constraints
	// For now, skip rigid body transformation in 3D mode

	// 3D Point cloud refinement
	neighbors := sys.cloud3D.RadiusSearch(fused.X, fused.Y, fused.Z, fused.R)
	sumX, sumY, sumZ := 0.0, 0.0, 0.0
	count := len(neighbors)
	for _, pt := range neighbors {
		sumX += pt.X
		sumY += pt.Y
		sumZ += pt.Z
	}
	finalX, finalY, finalZ := fused.X, fused.Y, fused.Z
	if count > 0 {
		finalX = sumX / float64(count)
		finalY = sumY / float64(count)
		finalZ = sumZ / float64(count)
	}

	// Output fused and refined position and orientation
	fmt.Printf("3D Fused position: (%.3f, %.3f, %.3f)\n", finalX, finalY, finalZ)
	
	// Output orientation information (convert quaternion to Euler angles for readability)
	if sys.imuCount > 0 {
		q := sys.orientations[0] // Use first IMU's orientation as example
		// Convert quaternion to Euler angles (roll, pitch, yaw)
		roll := math.Atan2(2*(q.W*q.X+q.Y*q.Z), 1-2*(q.X*q.X+q.Y*q.Y))
		pitch := math.Asin(2 * (q.W*q.Y - q.Z*q.X))
		yaw := math.Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))
		fmt.Printf("IMU 0 Orientation (roll, pitch, yaw): (%.3f, %.3f, %.3f) rad\n", roll, pitch, yaw)
	}
}
