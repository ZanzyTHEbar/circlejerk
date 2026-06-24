package internal

import (
	"math"
)

const epsilon = 1e-9 // Small tolerance for floating-point comparisons

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

// intersectTwoCircles finds the intersection points of two circles.
// Returns the number of intersection points (0, 1, or 2) and the points themselves.
func intersectTwoCircles(c1 Vec2, r1 float64, c2 Vec2, r2 float64) (int, Vec2, Vec2) {
	d := Distance2D(c1, c2)

	// Check for cases where circles do not intersect
	if d > r1+r2+epsilon || d < math.Abs(r1-r2)-epsilon || d < epsilon && math.Abs(r1-r2) > epsilon {
		return 0, Vec2{}, Vec2{} // No intersection or one contains the other without touching
	}

	// Calculate intersection points using formulas derived from law of cosines
	a := (r1*r1 - r2*r2 + d*d) / (2 * d)
	h := math.Sqrt(math.Max(0, r1*r1-a*a)) // Use Max(0, ...) to avoid NaN due to float errors

	// Find the midpoint between the intersection points
	x2 := c1.X + a*(c2.X-c1.X)/d
	y2 := c1.Y + a*(c2.Y-c1.Y)/d

	// Calculate the intersection points
	p1 := Vec2{
		X: x2 + h*(c2.Y-c1.Y)/d,
		Y: y2 - h*(c2.X-c1.X)/d,
	}
	p2 := Vec2{
		X: x2 - h*(c2.Y-c1.Y)/d,
		Y: y2 + h*(c2.X-c1.X)/d,
	}

	// Check for tangency (one intersection point)
	if d > r1+r2-epsilon || d < math.Abs(r1-r2)+epsilon || h < epsilon {
		return 1, p1, Vec2{} // Tangent
	}

	return 2, p1, p2 // Two intersection points
}

// isInsideAll checks if a point p is inside all circles defined by centers and radii.
func isInsideAll(p Vec2, centers []Vec2, radii []float64) bool {
	for i, c := range centers {
		if Distance2D(p, c) > radii[i]+epsilon {
			return false
		}
	}
	return true
}

// AllCirclesIntersectAtPoint checks if there exists a point p such that all circles (center, radius) contain p.
// It finds candidate points from intersections and containment, returning a feasible point if found.
// Returns (true, p) if such a point exists, else (false, zero).
func AllCirclesIntersectAtPoint(centers []Vec2, radii []float64) (bool, Vec2) {
	n := len(centers)
	if n == 0 {
		return false, Vec2{}
	}
	if n == 1 {
		return true, centers[0]
	}

	containedIndex := -1
	for i := 0; i < n; i++ {
		if isInsideAll(centers[i], centers, radii) && (containedIndex == -1 || radii[i] < radii[containedIndex]) {
			containedIndex = i
		}
	}
	if containedIndex != -1 {
		return true, centers[containedIndex]
	}

	var candidates []Vec2
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			count, p1, p2 := intersectTwoCircles(centers[i], radii[i], centers[j], radii[j])
			if count >= 1 {
				candidates = append(candidates, p1)
			}
			if count == 2 {
				candidates = append(candidates, p2)
			}
		}
	}

	valid := make([]Vec2, 0, len(candidates))
	for _, p := range candidates {
		if !isInsideAll(p, centers, radii) || containsVec2(valid, p) {
			continue
		}
		valid = append(valid, p)
	}
	if len(valid) == 1 {
		return true, valid[0]
	}
	if len(valid) > 1 {
		centroid := Vec2{}
		for _, p := range valid {
			centroid.X += p.X
			centroid.Y += p.Y
		}
		centroid.X /= float64(len(valid))
		centroid.Y /= float64(len(valid))
		if isInsideAll(centroid, centers, radii) {
			return true, centroid
		}
		return true, valid[0]
	}

	// 4. Fallback: Check the centroid of the original centers (for area intersections)
	originalCentroid := Vec2{}
	for _, c := range centers {
		originalCentroid.X += c.X
		originalCentroid.Y += c.Y
	}
	originalCentroid.X /= float64(n)
	originalCentroid.Y /= float64(n)
	if isInsideAll(originalCentroid, centers, radii) {
		return true, originalCentroid
	}

	// 5. No intersection found
	return false, Vec2{}
}

func containsVec2(points []Vec2, p Vec2) bool {
	for _, q := range points {
		if Distance2D(p, q) <= epsilon {
			return true
		}
	}
	return false
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
