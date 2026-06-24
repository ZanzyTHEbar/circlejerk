package internal

import (
	"testing"
	"time"
)

func TestDataAcquisitionProducesAlignedFrames(t *testing.T) {
	const imuCount = 4
	sync := NewSynchronizer()
	acq := NewDataAcquisition(imuCount, sync)
	acq.Start()
	defer acq.Stop()

	deadline := time.After(100 * time.Millisecond)
	for {
		if frames := sync.GetAlignedData(imuCount); len(frames) > 0 {
			if len(frames[0]) != imuCount {
				t.Fatalf("expected %d samples in frame, got %d", imuCount, len(frames[0]))
			}
			return
		}

		select {
		case <-deadline:
			t.Fatal("timed out waiting for aligned frame")
		case <-time.After(1 * time.Millisecond):
		}
	}
}
