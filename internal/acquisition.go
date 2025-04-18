package internal

import (
	"sync"
	"time"
)

// DataAcquisition handles the collection of data from multiple IMUs.
type DataAcquisition struct {
	sync     *Synchronizer
	imuCount int
	stopChan chan struct{}
	stopWg   sync.WaitGroup
	sync.Mutex
}

// NewDataAcquisition initializes a new DataAcquisition instance.
func NewDataAcquisition(imuCount int, sync *Synchronizer) *DataAcquisition {
	return &DataAcquisition{
		sync:     sync,
		imuCount: imuCount,
		stopChan: make(chan struct{}),
	}
}

// Start simulates the collection of data from the IMUs and sends it to the Synchronizer.
func (da *DataAcquisition) Start() {
	da.stopWg.Add(da.imuCount)
	for i := 0; i < da.imuCount; i++ {
		go func(imuID int) {
			defer da.stopWg.Done()
			// FIXME: Remove this simulation and replace with actual IMU data acquisition
			ticker := time.NewTicker(1 * time.Millisecond) // Simulate 1000Hz
			defer ticker.Stop()
			for {
				select {
				case <-ticker.C:
					// Simulate data acquisition
					data := IMUData{
						IMUID:           imuID,
						Timestamp:       time.Now(),                // Use precise timestamping if possible
						Acceleration:    [3]float64{0.0, 0.0, 0.0}, // Replace with actual data
						AngularVelocity: [3]float64{0.0, 0.0, 0.0}, // Replace with actual data
					}
					// Synchronization logic is handled by the Synchronizer type
					da.sync.AddData(data) // Send data to synchronizer
				case <-da.stopChan:
					return // Exit goroutine
				}
			}
		}(i)
	}
}

// Stop signals the data acquisition goroutines to stop.
func (da *DataAcquisition) Stop() {
	close(da.stopChan)
	da.stopWg.Wait()
}
