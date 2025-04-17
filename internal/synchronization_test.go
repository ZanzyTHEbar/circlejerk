package internal

import (
	"reflect"
	"sort"
	"testing"
	"time"
)

// Helper to sort IMUData within a frame by IMUID
func sortFrame(frame []IMUData) {
	sort.Slice(frame, func(i, j int) bool {
		return frame[i].IMUID < frame[j].IMUID
	})
}

// Helper to compare two slices of IMUData slices (frames)
func framesEqual(a, b [][]IMUData) bool {
	if len(a) != len(b) {
		return false
	}
	// Sort frames by timestamp for consistent comparison
	sort.Slice(a, func(i, j int) bool {
		if len(a[i]) == 0 || len(a[j]) == 0 {
			return len(a[i]) < len(a[j]) // Handle empty frames if necessary
		}
		return a[i][0].Timestamp.Before(a[j][0].Timestamp)
	})
	sort.Slice(b, func(i, j int) bool {
		if len(b[i]) == 0 || len(b[j]) == 0 {
			return len(b[i]) < len(b[j])
		}
		return b[i][0].Timestamp.Before(b[j][0].Timestamp)
	})

	for i := range a {
		if len(a[i]) != len(b[i]) {
			return false
		}
		// Sort data within each frame by IMUID
		sortFrame(a[i])
		sortFrame(b[i])
		if !reflect.DeepEqual(a[i], b[i]) {
			return false
		}
	}
	return true
}

func TestSynchronizer_AddAndGetAlignedData(t *testing.T) {
	sync := NewSynchronizer()
	imuCount := 2

	t1 := time.Now()
	t2 := t1.Add(1 * time.Millisecond)
	t3 := t1.Add(2 * time.Millisecond)

	data1_imu0 := IMUData{IMUID: 0, Timestamp: t1}
	data1_imu1 := IMUData{IMUID: 1, Timestamp: t1}

	data2_imu0 := IMUData{IMUID: 0, Timestamp: t2}
	// data2_imu1 missing initially

	data3_imu0 := IMUData{IMUID: 0, Timestamp: t3}
	data3_imu1 := IMUData{IMUID: 1, Timestamp: t3}

	// Add data out of order
	sync.AddData(data3_imu1)
	sync.AddData(data1_imu0)
	sync.AddData(data2_imu0)
	sync.AddData(data3_imu0)

	// Should get nothing yet, t1 and t3 are incomplete
	aligned := sync.GetAlignedData(imuCount)
	if len(aligned) != 0 {
		t.Fatalf("Expected 0 aligned frames initially, got %d", len(aligned))
	}

	// Complete frame for t1
	sync.AddData(data1_imu1)
	aligned = sync.GetAlignedData(imuCount)
	expected1 := [][]IMUData{{data1_imu0, data1_imu1}}
	if !framesEqual(aligned, expected1) {
		t.Errorf("Expected frame for t1 %v, got %v", expected1, aligned)
	}

	// Check if t1 frame was removed
	aligned = sync.GetAlignedData(imuCount)
	if len(aligned) != 0 {
		t.Errorf("Expected 0 aligned frames after getting t1, got %d", len(aligned))
	}

	// Add missing data for t2
	data2_imu1 := IMUData{IMUID: 1, Timestamp: t2}
	sync.AddData(data2_imu1)

	// Now get t2 and t3 (order might vary, handled by framesEqual)
	aligned = sync.GetAlignedData(imuCount)
	expected23 := [][]IMUData{
		{data2_imu0, data2_imu1},
		{data3_imu0, data3_imu1},
	}
	if !framesEqual(aligned, expected23) {
		t.Errorf("Expected frames for t2 and t3 %v, got %v", expected23, aligned)
	}

	// Check if all frames are gone
	aligned = sync.GetAlignedData(imuCount)
	if len(aligned) != 0 {
		t.Errorf("Expected 0 aligned frames after getting t2 and t3, got %d", len(aligned))
	}
}

func TestSynchronizer_ClearData(t *testing.T) {
	sync := NewSynchronizer()
	imuCount := 1
	t1 := time.Now()

	sync.AddData(IMUData{IMUID: 0, Timestamp: t1})

	// Ensure data is there (indirectly)
	if len(sync.dataMap) != 1 {
		t.Fatal("Data map should have 1 entry before clear")
	}

	sync.ClearData()

	if len(sync.dataMap) != 0 {
		t.Errorf("Data map should be empty after clear, size: %d", len(sync.dataMap))
	}

	// Verify GetAlignedData returns nothing after clear
	aligned := sync.GetAlignedData(imuCount)
	if len(aligned) != 0 {
		t.Errorf("Expected 0 aligned frames after clear, got %d", len(aligned))
	}
}
