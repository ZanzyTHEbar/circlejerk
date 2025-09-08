package internal

import (
	"math"
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
	OffsetZ float64 // Bias in the Z direction (3D support)
	ScaleX  float64 // Scale factor in the X direction
	ScaleY  float64 // Scale factor in the Y direction
	ScaleZ  float64 // Scale factor in the Z direction (3D support)
}

// NewIMU creates a new IMU with default calibration parameters.
func NewIMU() *IMU {
	return &IMU{
		OffsetX: 0.0,
		OffsetY: 0.0,
		OffsetZ: 0.0,
		ScaleX:  1.0,
		ScaleY:  1.0,
		ScaleZ:  1.0,
	}
}

// Point represents a 2D point in space.
type Point struct {
	X float64
	Y float64
}

// Point3D represents a 3D point in space.
type Point3D struct {
	X float64
	Y float64
	Z float64
}

// Quaternion represents orientation in 3D space for 6DOF support.
type Quaternion struct {
	W, X, Y, Z float64
}

// NewQuaternion creates a new identity quaternion.
func NewQuaternion() Quaternion {
	return Quaternion{W: 1.0, X: 0.0, Y: 0.0, Z: 0.0}
}

// Normalize normalizes the quaternion to unit length.
func (q *Quaternion) Normalize() {
	norm := math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	if norm > 0 {
		q.W /= norm
		q.X /= norm
		q.Y /= norm
		q.Z /= norm
	}
}

// Multiply multiplies two quaternions (q1 * q2).
func (q1 Quaternion) Multiply(q2 Quaternion) Quaternion {
	return Quaternion{
		W: q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z,
		X: q1.W*q2.X + q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y,
		Y: q1.W*q2.Y - q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X,
		Z: q1.W*q2.Z + q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W,
	}
}
