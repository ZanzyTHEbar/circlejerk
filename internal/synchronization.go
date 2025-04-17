package internal

import (
	"sort"
	"sync"
	"time"
)

// Synchronizer is responsible for synchronizing IMU data.
type Synchronizer struct {
	mu      sync.Mutex
	dataMap map[time.Time][]IMUData
}

// NewSynchronizer creates a new instance of Synchronizer.
func NewSynchronizer() *Synchronizer {
	return &Synchronizer{
		dataMap: make(map[time.Time][]IMUData),
	}
}

// AddData adds IMU data to the synchronizer.
func (s *Synchronizer) AddData(data IMUData) {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.dataMap[data.Timestamp] = append(s.dataMap[data.Timestamp], data)
}

// GetSynchronizedData retrieves synchronized IMU data.
func (s *Synchronizer) GetSynchronizedData() map[time.Time][]IMUData {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.dataMap
}

// ClearData clears the stored IMU data.
func (s *Synchronizer) ClearData() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.dataMap = make(map[time.Time][]IMUData)
}

// GetAlignedData returns a slice of IMUData slices, each containing one data point per IMU for timestamps where all IMUs have data.
// It processes timestamps chronologically and returns all completed frames up to the first incomplete one.
func (s *Synchronizer) GetAlignedData(imuCount int) [][]IMUData {
	s.mu.Lock()
	defer s.mu.Unlock()

	aligned := [][]IMUData{}

	// Get sorted timestamps
	timestamps := make([]time.Time, 0, len(s.dataMap))
	for ts := range s.dataMap {
		timestamps = append(timestamps, ts)
	}
	sort.Slice(timestamps, func(i, j int) bool {
		return timestamps[i].Before(timestamps[j])
	})

	// Process timestamps in order
	for _, ts := range timestamps {
		data := s.dataMap[ts]
		if len(data) == imuCount {
			// Frame is complete, add it to the result and remove from map
			aligned = append(aligned, data)
			delete(s.dataMap, ts)
		} else {
			// Found an incomplete frame, stop processing further timestamps
			break
		}
	}

	return aligned
}
