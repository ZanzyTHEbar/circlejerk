package main

import (
	"fmt"
	"log"

	"github.com/ZanzyTHEbar/imu-fusion/internal"
)

// TODO: further optimize, add tests, or implement advanced spatial queries,
// TODO: implement vectorization, SIMD, and goroutines with worker pools and batching
// TODO: Optimize integration via Runge Kutta or a closed-form solution (is that even possible?)

func main() {
	// Initialize the IMU fusion system
	imuSystem, err := internal.NewIMUFusionSystem(4)
	if err != nil {
		log.Fatalf("Failed to initialize IMU fusion system: %v", err)
	}

	// Process IMU data in real-time
	fmt.Println("IMU Fusion System is running...")
	imuSystem.Start()
}
