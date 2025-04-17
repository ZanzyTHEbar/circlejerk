package internal

import (
	"math"
)

// Uncertainty represents the uncertainty estimation for an IMU measurement.
type Uncertainty struct {
	NoiseLevel float64 // Noise level of the IMU
	IntegrationTime float64 // Time over which the acceleration is integrated
}

// NewUncertainty creates a new Uncertainty instance.
func NewUncertainty(noiseLevel, integrationTime float64) *Uncertainty {
	return &Uncertainty{
		NoiseLevel:     noiseLevel,
		IntegrationTime: integrationTime,
	}
}

// Estimate calculates the uncertainty based on the IMU noise specifications and integration time.
func (u *Uncertainty) Estimate() float64 {
	// Basic model for uncertainty estimation
	return u.NoiseLevel * math.Sqrt(u.IntegrationTime)
}