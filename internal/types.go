package internal

import (
	"time"
)

// IMUData represents the data structure for storing IMU measurements.
type IMUData struct {
	IMUID           int // ID of the originating IMU
	Timestamp       time.Time
	Acceleration    [3]float64 // x, y, z acceleration
	AngularVelocity [3]float64 // roll, pitch, yaw
}

// IMU represents an individual Inertial Measurement Unit with calibration.
type IMU struct {
	ID      int
	OffsetX float64 // Bias in the X direction
	OffsetY float64 // Bias in the Y direction
	ScaleX  float64 // Scale factor in the X direction
	ScaleY  float64 // Scale factor in the Y direction
}

// NewIMU creates a new IMU with default calibration parameters.
func NewIMU() *IMU {
	return &IMU{
		OffsetX: 0.0,
		OffsetY: 0.0,
		ScaleX:  1.0,
		ScaleY:  1.0,
	}
}

// Point represents a 2D point in space.
type Point struct {
	X float64
	Y float64
}
