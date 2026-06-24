package main

import (
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/ZanzyTHEbar/imu-fusion/internal"
)

func main() {
	// Initialize the IMU fusion system
	imuSystem, err := internal.NewIMUFusionSystem(4)
	if err != nil {
		log.Fatalf("Failed to initialize IMU fusion system: %v", err)
	}

	// Process IMU data in real-time
	fmt.Println("IMU Fusion System is running...")
	imuSystem.Start()

	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)
	<-stop
	imuSystem.Stop()
}
