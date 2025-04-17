package internal

import (
	"math"
	"testing"
)

// Helper function to check if two points are close
func pointsClose(p1, p2 Point, tol float64) bool {
	return math.Abs(p1.X-p2.X) < tol && math.Abs(p1.Y-p2.Y) < tol
}

// Helper function to check if two float values are close
func floatsClose(f1, f2, tol float64) bool {
	return math.Abs(f1-f2) < tol
}

func TestProcrustes(t *testing.T) {
	// Define a simple square as the target
	target := []Point{
		{0, 0},
		{1, 0},
		{1, 1},
		{0, 1},
	}

	// Define a rotated, scaled, and translated version as the source
	// Rotation: 90 degrees clockwise (cos(90)=0, sin(90)=1 -> R = [[0, 1], [-1, 0]])
	// Scale: 2.0
	// Translation: (3, 4)
	// Source points calculation:
	// p' = scale * R * p + translation
	// p0 = 2 * [0 1; -1 0] * [0; 0] + [3; 4] = [0; 0] + [3; 4] = [3; 4]
	// p1 = 2 * [0 1; -1 0] * [1; 0] + [3; 4] = 2 * [0; -1] + [3; 4] = [0; -2] + [3; 4] = [3; 2]
	// p2 = 2 * [0 1; -1 0] * [1; 1] + [3; 4] = 2 * [1; -1] + [3; 4] = [2; -2] + [3; 4] = [5; 2]
	// p3 = 2 * [0 1; -1 0] * [0; 1] + [3; 4] = 2 * [1; 0] + [3; 4] = [2; 0] + [3; 4] = [5; 4]
	source := []Point{
		{3, 4},
		{3, 2},
		{5, 2},
		{5, 4},
	}

	expectedCentroidTarget := Point{0.5, 0.5}
	// Procrustes finds the scale to transform SOURCE to TARGET.
	// Since source was created by scaling target by 2.0, the expected scale is 1/2.0 = 0.5
	expectedScale := 0.5
	tolerance := 1e-9

	aligned, centroidTarget, scale := Procrustes(source, target)

	// Check centroid
	if !pointsClose(centroidTarget, expectedCentroidTarget, tolerance) {
		t.Errorf("Expected centroid %v, got %v", expectedCentroidTarget, centroidTarget)
	}

	// Check scale (allow slightly larger tolerance for scale)
	if !floatsClose(scale, expectedScale, 1e-6) {
		t.Errorf("Expected scale approx %f, got %f", expectedScale, scale)
	}

	// Check if aligned points match the target points
	if len(aligned) != len(target) {
		t.Fatalf("Expected %d aligned points, got %d", len(target), len(aligned))
	}
	for i := range target {
		// Note: Procrustes aligns source TO target. The returned 'aligned' points
		// are the *source* points after applying the calculated transformation.
		// They should now be close to the original target points.

		// The returned `aligned` points are the transformed source points.
		// Let's check if they are close to the original target points.
		if !pointsClose(aligned[i], target[i], tolerance) {
			t.Errorf("Expected aligned point %d to be close to target %v, got %v", i, target[i], aligned[i])
		}
	}
}

// TODO: Add tests for edge cases (e.g., colinear points, identical point sets, insufficient points)
