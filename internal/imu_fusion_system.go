package internal

import (
	"fmt"
	"time"
)

// IMUFusionSystem is the main struct orchestrating the fusion pipeline.
type IMUFusionSystem struct {
	acq        *DataAcquisition
	sync       *Synchronizer
	calib      []*IMU
	cloud      *PointCloud
	positions  []Point   // per-IMU position state
	velocities []Point   // per-IMU velocity state
	lastTime   time.Time // last timestamp for integration
	noiseLevel float64   // IMU noise level for uncertainty calculation
	reference  []Point   // reference geometry for rigid transform
	imuCount   int       // number of IMUs
}

// NewIMUFusionSystem initializes the IMU fusion system.
func NewIMUFusionSystem(imuCount int) (*IMUFusionSystem, error) {
	sync := NewSynchronizer()
	acq := NewDataAcquisition(imuCount, sync) // Pass synchronizer to acquisition
	calib := make([]*IMU, imuCount)
	for i := 0; i < imuCount; i++ {
		calib[i] = NewIMU()
		calib[i].ID = i // Assign ID
	}
	cloud := NewPointCloud()
	positions := make([]Point, imuCount)
	velocities := make([]Point, imuCount)
	now := time.Now()
	noise := 0.1 // default noise level
	d := 1.0
	ref := []Point{{0, 0}, {d, 0}, {d, d}, {0, d}} // Assuming 4 IMUs for ref geometry
	if imuCount != 4 {
		// TODO: Handle reference geometry for different imu counts
		fmt.Println("Warning: Reference geometry is hardcoded for 4 IMUs.")
	}

	return &IMUFusionSystem{
		acq:        acq,
		sync:       sync,
		calib:      calib,
		cloud:      cloud,
		positions:  positions,
		velocities: velocities,
		lastTime:   now,
		noiseLevel: noise,
		reference:  ref,
		imuCount:   imuCount,
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
			fmt.Printf("Fused position: (%.3f, %.3f)\n", finalX, finalY)
		}
	}
}
