package internal

import (
	"fmt"
	"math"
	"sort" // Added for sorting contained circles
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
// Returns (true, p) if such a point exists, else (false, zero).
func AllCirclesIntersectAtPoint(centers []Vec2, radii []float64) (bool, Vec2) {
	n := len(centers)
	if n == 0 {
		return false, Vec2{}
	}
	if n == 1 {
		return true, centers[0]
	}

	// 1. Generate candidate points from pairwise intersections and immediately check if they are inside all circles
	var validIntersectionPoints []Vec2
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			count, p1, p2 := intersectTwoCircles(centers[i], radii[i], centers[j], radii[j])
			if count >= 1 {
				if isInsideAll(p1, centers, radii) {
					validIntersectionPoints = append(validIntersectionPoints, p1)
				}
			}
			if count == 2 {
				if isInsideAll(p2, centers, radii) {
					validIntersectionPoints = append(validIntersectionPoints, p2)
				}
			}
		}
	}

	// 2. Process valid boundary intersection points
	if len(validIntersectionPoints) > 0 {
		centroid := Vec2{}
		seen := make(map[string]bool)
		uniqueValidPointsCount := 0
		var firstUniquePoint Vec2
		firstPointSet := false

		// Deduplicate and count unique points
		for _, p := range validIntersectionPoints {
			key := fmt.Sprintf("%.9f,%.9f", p.X, p.Y)
			if !seen[key] {
				seen[key] = true
				uniqueValidPointsCount++
				if !firstPointSet {
					firstUniquePoint = p
					firstPointSet = true
				}
			}
		}

		// If exactly one unique valid point, return it (handles tangent/3-circle intersection)
		if uniqueValidPointsCount == 1 && firstPointSet {
			return true, firstUniquePoint
		}

		// If multiple unique points, calculate and check centroid
		if uniqueValidPointsCount > 1 {
		    // Recalculate centroid only from unique points
		    centroid = Vec2{}
		    for _, p := range validIntersectionPoints {
		        key := fmt.Sprintf("%.9f,%.9f", p.X, p.Y)
		        if seen[key] { // Check if it's a unique point we counted
		            centroid.X += p.X
		            centroid.Y += p.Y
		            // Mark as used for centroid calculation to avoid re-adding
		            // TODO: Note: This simple approach assumes the map iteration order is consistent enough
		            // or that adding the same point multiple times if it appeared multiple times
		            // in the original list before deduplication is acceptable for centroid.
		            // A cleaner way might be to store unique points in a separate slice.
		            // Let's refine this: store unique points properly.
		        }
            }
            // Correct way: Collect unique points first
            uniquePoints := make([]Vec2, 0, uniqueValidPointsCount)
            processedKeys := make(map[string]bool)
            for _, p := range validIntersectionPoints {
                key := fmt.Sprintf("%.9f,%.9f", p.X, p.Y)
                if seen[key] && !processedKeys[key] {
                    uniquePoints = append(uniquePoints, p)
                    processedKeys[key] = true
                }
            }
            // Calculate centroid from the actual unique points
            centroid = Vec2{}
            for _, p := range uniquePoints {
                centroid.X += p.X
                centroid.Y += p.Y
            }
			centroid.X /= float64(uniqueValidPointsCount)
			centroid.Y /= float64(uniqueValidPointsCount)

			// Check if the centroid is valid
			if isInsideAll(centroid, centers, radii) {
				return true, centroid
			}
			// If centroid invalid, but we had multiple points, fall back to the first unique one found
			// This might happen if the intersection is an arc and the centroid falls outside.
			if firstPointSet {
			    return true, firstUniquePoint
			}
		}
	}

	// 3. If no valid boundary points, check for contained circles
	type containedCircleInfo struct {
		center Vec2
		radius float64
	}
	var containedCircles []containedCircleInfo
	for i := 0; i < n; i++ {
		if isInsideAll(centers[i], centers, radii) {
			containedCircles = append(containedCircles, containedCircleInfo{centers[i], radii[i]})
		}
	}

	if len(containedCircles) > 0 {
		// Sort by radius (ascending) to find the smallest contained circle
		sort.Slice(containedCircles, func(i, j int) bool {
			return containedCircles[i].radius < containedCircles[j].radius
		})
		// Return the center of the smallest circle whose center is inside all others
		return true, containedCircles[0].center
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
