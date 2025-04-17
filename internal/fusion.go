package internal

import (
	"math"
)

//! Use GeometricFusion2D as the main entry point for geometric fusion.
//! Example usage:
//!   alpha, fused := GeometricFusion2D(positions)
//! This finds the minimal expansion factor alpha and the fused position where all circles intersect.

// Position represents a 2D position with uncertainty.
type Position struct {
	X float64
	Y float64
	R float64 // Uncertainty radius
}

// Vec2 is a simple 2D vector.
type Vec2 struct {
	X, Y float64
}

// Distance2D computes the Euclidean distance between two 2D points.
func Distance2D(a, b Vec2) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
}

// AllCirclesIntersectAtPoint checks if there exists a point p such that all circles (center, radius) contain p.
// Returns (true, p) if such a point exists, else (false, zero).
func AllCirclesIntersectAtPoint(centers []Vec2, radii []float64) (bool, Vec2) {
	// Robust grid search for intersection region
	// 1. Find bounding box
	minX, maxX := centers[0].X, centers[0].X
	minY, maxY := centers[0].Y, centers[0].Y
	for _, c := range centers {
		if c.X < minX {
			minX = c.X
		}
		if c.X > maxX {
			maxX = c.X
		}
		if c.Y < minY {
			minY = c.Y
		}
		if c.Y > maxY {
			maxY = c.Y
		}
	}
	// 2. Grid search with step size
	step := 0.01 * (maxX - minX + maxY - minY) / 2.0
	best := Vec2{}
	found := false
	minSum := 1e12

	// TODO: Refactor to vectorize
	for x := minX; x <= maxX; x += step {
		for y := minY; y <= maxY; y += step {
			inside := true
			sum := 0.0
			for i, c := range centers {
				d := Distance2D(Vec2{X: x, Y: y}, c)
				if d > radii[i] {
					inside = false
					break
				}
				sum += d
			}
			if inside && sum < minSum {
				minSum = sum
				best = Vec2{X: x, Y: y}
				found = true
			}
		}
	}
	return found, best
}

// GeometricFusion2D finds the minimal alpha >= 1 such that all expanded circles intersect at some point.
// Returns (alpha, fused position).
func GeometricFusion2D(positions []Position) (float64, Position) {
	centers := make([]Vec2, len(positions))
	radii := make([]float64, len(positions))
	for i, pos := range positions {
		centers[i] = Vec2{X: pos.X, Y: pos.Y}
		radii[i] = pos.R
	}
	alphaMin, alphaMax := 1.0, 10.0
	var fused Vec2
	for alphaMax-alphaMin > 1e-4 {
		alpha := 0.5 * (alphaMin + alphaMax)
		expanded := make([]float64, len(radii))
		for i := range radii {
			expanded[i] = alpha * radii[i]
		}
		ok, p := AllCirclesIntersectAtPoint(centers, expanded)
		if ok {
			alphaMax = alpha
			fused = p
		} else {
			alphaMin = alpha
		}
	}
	return alphaMax, Position{X: fused.X, Y: fused.Y, R: alphaMax}
}

// CircleIntersection checks if two circles intersect.
func CircleIntersection(p1, r1, p2, r2 float64) bool {
	dx := p2 - p1
	dy := r2 - r1
	distanceSquared := dx*dx + dy*dy
	radiusSum := r1 + r2
	return distanceSquared <= radiusSum*radiusSum
}
