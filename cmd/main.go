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
	// Initialize the IMU fusion system in 2D mode (backward compatibility)
	fmt.Println("=== 2D IMU Fusion System Demo ===")
	imuSystem2D, err := internal.NewIMUFusionSystem(4)
	if err != nil {
		log.Fatalf("Failed to initialize 2D IMU fusion system: %v", err)
	}

	// Process IMU data in real-time (2D mode)
	fmt.Println("2D IMU Fusion System is running...")
	go imuSystem2D.Start()

	// Initialize the IMU fusion system in 3D mode with 6DOF
	fmt.Println("\n=== 3D IMU Fusion System with 6DOF Demo ===")
	imuSystem3D, err := internal.NewIMUFusionSystemWithMode(4, true)
	if err != nil {
		log.Fatalf("Failed to initialize 3D IMU fusion system: %v", err)
	}

	// Process IMU data in real-time (3D mode with 6DOF)
	fmt.Println("3D IMU Fusion System with 6DOF is running...")
	go imuSystem3D.Start()

	// Keep the application running for demonstration
	// In a real application, you would have proper shutdown handling
	fmt.Println("\nBoth systems are running. Press Ctrl+C to exit.")
	select {} // Block forever
}
