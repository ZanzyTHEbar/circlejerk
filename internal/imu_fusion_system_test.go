package internal

import (
	"testing"
	"time"
)

func TestIMUFusionSystemStopReturns(t *testing.T) {
	sys, err := NewIMUFusionSystem(4)
	if err != nil {
		t.Fatalf("NewIMUFusionSystem failed: %v", err)
	}
	sys.Start()

	done := make(chan struct{})
	go func() {
		sys.Stop()
		close(done)
	}()

	select {
	case <-done:
	case <-time.After(100 * time.Millisecond):
		t.Fatal("Stop did not return")
	}
}
