package internal

import (
	"fmt"
	"sync"
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
	imuCount   int       // number of IMUs
	stopChan   chan struct{}
	stopWg     sync.WaitGroup
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
	return &IMUFusionSystem{
		acq:        acq,
		sync:       sync,
		calib:      calib,
		cloud:      cloud,
		positions:  positions,
		velocities: velocities,
		lastTime:   now,
		noiseLevel: noise,
		imuCount:   imuCount,
		stopChan:   make(chan struct{}),
	}, nil
}

// Start starts the data acquisition and processing loop.
func (sys *IMUFusionSystem) Start() {
	sys.acq.Start()
	sys.stopWg.Add(1)
	go sys.processDataLoop()
}

// Stop stops the data acquisition and processing.
func (sys *IMUFusionSystem) Stop() {
	close(sys.stopChan)
	sys.acq.Stop()
	sys.stopWg.Wait()
}

// processDataLoop runs the main fusion logic.
func (sys *IMUFusionSystem) processDataLoop() {
	defer sys.stopWg.Done()
	for {
		select {
		case <-sys.stopChan:
			return
		default:
		}

		// Get aligned data frames from the synchronizer
		alignedFrames := sys.sync.GetAlignedData(sys.imuCount)
		if len(alignedFrames) == 0 {
			select {
			case <-sys.stopChan:
				return
			case <-time.After(1 * time.Millisecond):
			}
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
