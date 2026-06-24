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
	da.stopWg.Add(1)
	go func() {
		defer da.stopWg.Done()
		ticker := time.NewTicker(1 * time.Millisecond) // Simulate 1000Hz frames.
		defer ticker.Stop()
		for {
			select {
			case ts := <-ticker.C:
				for imuID := 0; imuID < da.imuCount; imuID++ {
					data := IMUData{
						IMUID:           imuID,
						Timestamp:       ts,
						Acceleration:    [3]float64{},
						AngularVelocity: [3]float64{},
					}
					da.sync.AddData(data)
				}
			case <-da.stopChan:
				return
			}
		}
	}()
}

// Stop signals the data acquisition goroutines to stop.
func (da *DataAcquisition) Stop() {
	close(da.stopChan)
	da.stopWg.Wait()
}
