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

// Intersects checks if two circles intersect.
func (c1 *Circle) Intersects(c2 *Circle) bool {
	distance := math.Sqrt(math.Pow(c1.X-c2.X, 2) + math.Pow(c1.Y-c2.Y, 2))
	return distance <= (c1.Radius + c2.Radius)
}

// Expand expands the radius of the circle by a given factor.
func (c *Circle) Expand(factor float64) {
	c.Radius *= factor
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