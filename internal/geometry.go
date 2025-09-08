package internal

import (
	"math"
)

// Circle represents a circle in 2D space with a center point and a radius.
type Circle struct {
	X      float64 // X coordinate of the center
	Y      float64 // Y coordinate of the center
	Radius float64 // Radius of the circle
}

// Sphere represents a sphere in 3D space with a center point and a radius.
type Sphere struct {
	X      float64 // X coordinate of the center
	Y      float64 // Y coordinate of the center
	Z      float64 // Z coordinate of the center
	Radius float64 // Radius of the sphere
}

// Intersects checks if two circles intersect.
func (c1 *Circle) Intersects(c2 *Circle) bool {
	distance := math.Sqrt(math.Pow(c1.X-c2.X, 2) + math.Pow(c1.Y-c2.Y, 2))
	return distance <= (c1.Radius + c2.Radius)
}

// Intersects checks if two spheres intersect.
func (s1 *Sphere) Intersects(s2 *Sphere) bool {
	distance := math.Sqrt(math.Pow(s1.X-s2.X, 2) + math.Pow(s1.Y-s2.Y, 2) + math.Pow(s1.Z-s2.Z, 2))
	return distance <= (s1.Radius + s2.Radius)
}

// Expand expands the radius of the circle by a given factor.
func (c *Circle) Expand(factor float64) {
	c.Radius *= factor
}

// Expand expands the radius of the sphere by a given factor.
func (s *Sphere) Expand(factor float64) {
	s.Radius *= factor
}

// FusedPosition calculates the weighted average position of multiple circles based on their uncertainties.
func FusedPosition(circles []Circle, uncertainties []float64) (float64, float64) {
	var weightedX, weightedY, weightSum float64

	for i, circle := range circles {
		if uncertainties[i] > 0 {
			weight := 1 / uncertainties[i]
			weightedX += circle.X * weight
			weightedY += circle.Y * weight
			weightSum += weight
		}
	}

	if weightSum > 0 {
		return weightedX / weightSum, weightedY / weightSum
	}
	return 0, 0 // Return origin if no valid circles
}

// FusedPosition3D calculates the weighted average position of multiple spheres based on their uncertainties.
func FusedPosition3D(spheres []Sphere, uncertainties []float64) (float64, float64, float64) {
	var weightedX, weightedY, weightedZ, weightSum float64

	for i, sphere := range spheres {
		if uncertainties[i] > 0 {
			weight := 1 / uncertainties[i]
			weightedX += sphere.X * weight
			weightedY += sphere.Y * weight
			weightedZ += sphere.Z * weight
			weightSum += weight
		}
	}

	if weightSum > 0 {
		return weightedX / weightSum, weightedY / weightSum, weightedZ / weightSum
	}
	return 0, 0, 0 // Return origin if no valid spheres
}