package internal

import (
	"math"
)

// Calibrate performs calibration on the IMU using provided raw data.
// It adjusts the offset and scale based on the average of the measurements.
func (imu *IMU) Calibrate(rawData [][]float64) {
	var sumX, sumY, sumZ float64
	count := float64(len(rawData))

	for _, data := range rawData {
		sumX += data[0]
		sumY += data[1]
		if len(data) > 2 { // Support 3D calibration if Z data is available
			sumZ += data[2]
		}
	}

	// Calculate the average measurements
	avgX := sumX / count
	avgY := sumY / count
	avgZ := sumZ / count

	// Set the offsets based on the average
	imu.OffsetX = avgX
	imu.OffsetY = avgY
	imu.OffsetZ = avgZ

	// For scale calibration, we can assume a known reference value
	// Here we simply set scale factors to 1 for simplicity
	imu.ScaleX = 1.0
	imu.ScaleY = 1.0
	imu.ScaleZ = 1.0
}

// ApplyCalibration applies the calibration parameters to raw IMU measurements.
func (imu *IMU) ApplyCalibration(rawX, rawY float64) (float64, float64) {
	calibratedX := (rawX - imu.OffsetX) * imu.ScaleX
	calibratedY := (rawY - imu.OffsetY) * imu.ScaleY
	return calibratedX, calibratedY
}

// ApplyCalibration3D applies the calibration parameters to raw 3D IMU measurements.
func (imu *IMU) ApplyCalibration3D(rawX, rawY, rawZ float64) (float64, float64, float64) {
	calibratedX := (rawX - imu.OffsetX) * imu.ScaleX
	calibratedY := (rawY - imu.OffsetY) * imu.ScaleY
	calibratedZ := (rawZ - imu.OffsetZ) * imu.ScaleZ
	return calibratedX, calibratedY, calibratedZ
}

// CalculateError computes the calibration error based on expected and measured values.
func CalculateError(expectedX, expectedY, measuredX, measuredY float64) float64 {
	errorX := expectedX - measuredX
	errorY := expectedY - measuredY
	return math.Sqrt(errorX*errorX + errorY*errorY) // Euclidean distance
}

// CalculateError3D computes the 3D calibration error based on expected and measured values.
func CalculateError3D(expectedX, expectedY, expectedZ, measuredX, measuredY, measuredZ float64) float64 {
	errorX := expectedX - measuredX
	errorY := expectedY - measuredY
	errorZ := expectedZ - measuredZ
	return math.Sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ) // 3D Euclidean distance
}
