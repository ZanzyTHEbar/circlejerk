package internal

import (
	"math"
	"testing"
)

// Re-use helper from procrustes_test.go if possible, or redefine here
// func pointsClose(p1, p2 Point, tol float64) bool {
// 	return math.Abs(p1.X-p2.X) < tol && math.Abs(p1.Y-p2.Y) < tol
// }
// func floatsClose(f1, f2, tol float64) bool {
// 	return math.Abs(f1-f2) < tol
// }

func TestAllCirclesIntersectAtPoint(t *testing.T) {
	tolerance := 0.1 // Grid search might not be perfectly accurate

	tests := []struct {
		name      string
		centers   []Vec2
		radii     []float64
		expectOk  bool
		expectPos Vec2 // Expected position if expectOk is true
	}{
		{
			name: "Simple Intersection",
			centers: []Vec2{
				{0, 0},
				{2, 0},
			},
			radii:     []float64{1.1, 1.1},
			expectOk:  true,
			expectPos: Vec2{1, 0}, // Intersection is around (1, y)
		},
		{
			name: "No Intersection",
			centers: []Vec2{
				{0, 0},
				{3, 0},
			},
			radii:     []float64{1, 1},
			expectOk:  false,
			expectPos: Vec2{}, // Position doesn't matter
		},
		{
			name: "Tangent Circles",
			centers: []Vec2{
				{0, 0},
				{2, 0},
			},
			radii:     []float64{1, 1},
			expectOk:  true,
			expectPos: Vec2{1, 0}, // Tangent point
		},
		{
			name: "One Contains Another",
			centers: []Vec2{
				{0, 0},
				{0.5, 0},
			},
			radii:     []float64{2, 0.5},
			expectOk:  true,
			expectPos: Vec2{0.5, 0}, // Center of smaller circle is a valid point
		},
		{
			name: "Three Circles Intersecting",
			centers: []Vec2{
				{0, 0},
				{2, 0},
				{1, 1.732}, // Equilateral triangle vertices
			},
			radii:     []float64{1.1, 1.1, 1.1},
			expectOk:  true,
			expectPos: Vec2{1, 0.577}, // Centroid of the triangle
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			ok, pos := AllCirclesIntersectAtPoint(tt.centers, tt.radii)
			if ok != tt.expectOk {
				t.Errorf("Expected ok=%v, got ok=%v", tt.expectOk, ok)
			}
			if ok && tt.expectOk {
				// Use Vec2 version of pointsClose if available, otherwise compare fields
				if math.Abs(pos.X-tt.expectPos.X) > tolerance || math.Abs(pos.Y-tt.expectPos.Y) > tolerance {
					t.Errorf("Expected position close to %v, got %v", tt.expectPos, pos)
				}
			}
		})
	}
}

func TestGeometricFusion2D(t *testing.T) {
	tolerance := 1e-3   // Binary search tolerance
	posTolerance := 0.1 // Grid search tolerance from AllCirclesIntersectAtPoint

	tests := []struct {
		name        string
		positions   []Position
		expectAlpha float64  // Expected alpha value
		expectPos   Position // Expected fused position (approx)
	}{
		{
			name: "Already Intersecting",
			positions: []Position{
				{X: 0, Y: 0, R: 1.1},
				{X: 2, Y: 0, R: 1.1},
			},
			expectAlpha: 1.0,
			expectPos:   Position{X: 1, Y: 0, R: 1.0}, // R should be alpha
		},
		{
			name: "Needs Expansion",
			positions: []Position{
				{X: 0, Y: 0, R: 1.0},
				{X: 3, Y: 0, R: 1.0},
			},
			expectAlpha: 1.5, // d=3, r1+r2=2. Need alpha*(r1+r2) = d => alpha*2 = 3 => alpha=1.5
			expectPos:   Position{X: 1.5, Y: 0, R: 1.5},
		},
		{
			name: "Tangent Case",
			positions: []Position{
				{X: 0, Y: 0, R: 1.0},
				{X: 2, Y: 0, R: 1.0},
			},
			expectAlpha: 1.0,
			expectPos:   Position{X: 1.0, Y: 0, R: 1.0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			alpha, fusedPos := GeometricFusion2D(tt.positions)

			if !floatsClose(alpha, tt.expectAlpha, tolerance) {
				t.Errorf("Expected alpha close to %f, got %f", tt.expectAlpha, alpha)
			}

			// Check fused position (using looser tolerance due to grid search)
			if math.Abs(fusedPos.X-tt.expectPos.X) > posTolerance || math.Abs(fusedPos.Y-tt.expectPos.Y) > posTolerance {
				t.Errorf("Expected fused position close to (%f, %f), got (%f, %f)", tt.expectPos.X, tt.expectPos.Y, fusedPos.X, fusedPos.Y)
			}
			// Check fused radius (should be alpha)
			if !floatsClose(fusedPos.R, alpha, tolerance) {
				t.Errorf("Expected fused radius R to be alpha (%f), got %f", alpha, fusedPos.R)
			}
		})
	}
}
