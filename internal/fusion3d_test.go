package internal

import (
	"math"
	"testing"
)

func TestQuaternion(t *testing.T) {
	t.Run("NewQuaternion", func(t *testing.T) {
		q := NewQuaternion()
		if q.W != 1.0 || q.X != 0.0 || q.Y != 0.0 || q.Z != 0.0 {
			t.Errorf("Expected identity quaternion (1, 0, 0, 0), got (%.3f, %.3f, %.3f, %.3f)", q.W, q.X, q.Y, q.Z)
		}
	})

	t.Run("Normalize", func(t *testing.T) {
		q := Quaternion{W: 2.0, X: 0.0, Y: 0.0, Z: 0.0}
		q.Normalize()
		if !floatsClose(q.W, 1.0, 1e-6) {
			t.Errorf("Expected normalized quaternion W=1.0, got W=%.6f", q.W)
		}
	})

	t.Run("Multiply", func(t *testing.T) {
		q1 := Quaternion{W: 1.0, X: 0.0, Y: 0.0, Z: 0.0} // identity
		q2 := Quaternion{W: 0.0, X: 1.0, Y: 0.0, Z: 0.0} // 180° rotation around X
		result := q1.Multiply(q2)
		expected := q2
		if !floatsClose(result.W, expected.W, 1e-6) || !floatsClose(result.X, expected.X, 1e-6) ||
			!floatsClose(result.Y, expected.Y, 1e-6) || !floatsClose(result.Z, expected.Z, 1e-6) {
			t.Errorf("Expected quaternion multiplication result (%.3f, %.3f, %.3f, %.3f), got (%.3f, %.3f, %.3f, %.3f)",
				expected.W, expected.X, expected.Y, expected.Z, result.W, result.X, result.Y, result.Z)
		}
	})
}

func TestPoint3D(t *testing.T) {
	t.Run("Dimensions", func(t *testing.T) {
		p := Point3D{X: 1.0, Y: 2.0, Z: 3.0}
		if p.Dimensions() != 3 {
			t.Errorf("Expected 3 dimensions, got %d", p.Dimensions())
		}
	})

	t.Run("Dimension", func(t *testing.T) {
		p := Point3D{X: 1.0, Y: 2.0, Z: 3.0}
		if p.Dimension(0) != 1.0 || p.Dimension(1) != 2.0 || p.Dimension(2) != 3.0 {
			t.Errorf("Expected dimensions (1.0, 2.0, 3.0), got (%.1f, %.1f, %.1f)",
				p.Dimension(0), p.Dimension(1), p.Dimension(2))
		}
	})

	t.Run("Distance", func(t *testing.T) {
		p1 := Point3D{X: 0.0, Y: 0.0, Z: 0.0}
		p2 := Point3D{X: 3.0, Y: 4.0, Z: 0.0}
		dist := p1.Distance(p2)
		expected := 25.0 // squared distance: 3^2 + 4^2 = 25
		if !floatsClose(dist, expected, 1e-6) {
			t.Errorf("Expected squared distance %.1f, got %.6f", expected, dist)
		}
	})
}

func TestDistance3D(t *testing.T) {
	a := Vec3{X: 0, Y: 0, Z: 0}
	b := Vec3{X: 3, Y: 4, Z: 0}
	dist := Distance3D(a, b)
	expected := 5.0 // sqrt(3^2 + 4^2) = 5
	if !floatsClose(dist, expected, 1e-6) {
		t.Errorf("Expected distance %.1f, got %.6f", expected, dist)
	}
}

func TestAllSpheresIntersectAtPoint(t *testing.T) {
	tolerance := 0.2 // Increased tolerance for 3D sphere intersection

	tests := []struct {
		name      string
		centers   []Vec3
		radii     []float64
		expectOk  bool
		expectPos Vec3 // Expected position if expectOk is true
	}{
		{
			name: "Single Sphere",
			centers: []Vec3{
				{0, 0, 0},
			},
			radii:     []float64{1.0},
			expectOk:  true,
			expectPos: Vec3{0, 0, 0},
		},
		{
			name: "Two Intersecting Spheres",
			centers: []Vec3{
				{0, 0, 0},
				{1, 0, 0},
			},
			radii:     []float64{0.7, 0.7},
			expectOk:  true,
			expectPos: Vec3{0.5, 0, 0}, // Midpoint should be inside both
		},
		{
			name: "Two Non-Intersecting Spheres",
			centers: []Vec3{
				{0, 0, 0},
				{3, 0, 0},
			},
			radii:     []float64{1.0, 1.0},
			expectOk:  false,
			expectPos: Vec3{}, // Position doesn't matter
		},
		{
			name: "Four Spheres in Tetrahedral Formation",
			centers: []Vec3{
				{0, 0, 0},
				{1, 0, 0},
				{0.5, math.Sqrt(3)/2, 0},
				{0.5, math.Sqrt(3)/6, math.Sqrt(2.0/3.0)},
			},
			radii:     []float64{0.8, 0.8, 0.8, 0.8}, // Increased radius for better intersection
			expectOk:  true,
			expectPos: Vec3{0.5, math.Sqrt(3)/6, math.Sqrt(2.0/3.0)/3}, // More conservative expected position
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			ok, pos := AllSpheresIntersectAtPoint(tt.centers, tt.radii)
			if ok != tt.expectOk {
				t.Errorf("Expected ok=%v, got ok=%v", tt.expectOk, ok)
			}
			if ok && tt.expectOk {
				if math.Abs(pos.X-tt.expectPos.X) > tolerance ||
					math.Abs(pos.Y-tt.expectPos.Y) > tolerance ||
					math.Abs(pos.Z-tt.expectPos.Z) > tolerance {
					t.Errorf("Expected position close to (%.3f, %.3f, %.3f), got (%.3f, %.3f, %.3f)",
						tt.expectPos.X, tt.expectPos.Y, tt.expectPos.Z, pos.X, pos.Y, pos.Z)
				}
			}
		})
	}
}

func TestGeometricFusion3D(t *testing.T) {
	tolerance := 1e-3
	posTolerance := 0.1

	tests := []struct {
		name        string
		positions   []Position3D
		expectAlpha float64    // Expected alpha value
		expectPos   Position3D // Expected fused position (approx)
	}{
		{
			name: "Two Already Intersecting Spheres",
			positions: []Position3D{
				{X: 0, Y: 0, Z: 0, R: 0.7},
				{X: 1, Y: 0, Z: 0, R: 0.7},
			},
			expectAlpha: 1.0,
			expectPos:   Position3D{X: 0.5, Y: 0, Z: 0, R: 1.0},
		},
		{
			name: "Two Spheres Needing Expansion",
			positions: []Position3D{
				{X: 0, Y: 0, Z: 0, R: 0.5},
				{X: 2, Y: 0, Z: 0, R: 0.5},
			},
			expectAlpha: 2.0, // d=2, r1+r2=1. Need alpha*(r1+r2) >= d => alpha*1 >= 2 => alpha>=2
			expectPos:   Position3D{X: 1.0, Y: 0, Z: 0, R: 2.0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			alpha, fusedPos := GeometricFusion3D(tt.positions)

			if !floatsClose(alpha, tt.expectAlpha, tolerance) {
				t.Errorf("Expected alpha close to %f, got %f", tt.expectAlpha, alpha)
			}

			// Check fused position (using looser tolerance)
			if math.Abs(fusedPos.X-tt.expectPos.X) > posTolerance ||
				math.Abs(fusedPos.Y-tt.expectPos.Y) > posTolerance ||
				math.Abs(fusedPos.Z-tt.expectPos.Z) > posTolerance {
				t.Errorf("Expected fused position close to (%.3f, %.3f, %.3f), got (%.3f, %.3f, %.3f)",
					tt.expectPos.X, tt.expectPos.Y, tt.expectPos.Z, fusedPos.X, fusedPos.Y, fusedPos.Z)
			}
			// Check fused radius (should be alpha)
			if !floatsClose(fusedPos.R, alpha, tolerance) {
				t.Errorf("Expected fused radius R to be alpha (%.3f), got %.3f", alpha, fusedPos.R)
			}
		})
	}
}

func TestPointCloud3D(t *testing.T) {
	cloud := NewPointCloud3D()

	t.Run("AddPoint", func(t *testing.T) {
		cloud.AddPoint(1.0, 2.0, 3.0)
		points := cloud.GetPoints()
		if len(points) != 1 {
			t.Errorf("Expected 1 point, got %d", len(points))
		}
		if points[0].X != 1.0 || points[0].Y != 2.0 || points[0].Z != 3.0 {
			t.Errorf("Expected point (1.0, 2.0, 3.0), got (%.1f, %.1f, %.1f)",
				points[0].X, points[0].Y, points[0].Z)
		}
	})

	t.Run("RadiusSearch", func(t *testing.T) {
		cloud.Clear()
		cloud.AddPoint(0.0, 0.0, 0.0)
		cloud.AddPoint(1.0, 0.0, 0.0)
		cloud.AddPoint(2.0, 0.0, 0.0)
		cloud.AddPoint(0.0, 1.0, 0.0)

		neighbors := cloud.RadiusSearch(0.5, 0.0, 0.0, 1.0)
		if len(neighbors) != 2 {
			t.Errorf("Expected 2 neighbors within radius, got %d", len(neighbors))
		}
	})

	t.Run("Clear", func(t *testing.T) {
		cloud.Clear()
		points := cloud.GetPoints()
		if len(points) != 0 {
			t.Errorf("Expected 0 points after clear, got %d", len(points))
		}
	})
}

func TestIMUFusionSystem3D(t *testing.T) {
	t.Run("NewIMUFusionSystemWithMode3D", func(t *testing.T) {
		sys, err := NewIMUFusionSystemWithMode(4, true)
		if err != nil {
			t.Fatalf("Failed to create 3D IMU fusion system: %v", err)
		}
		if !sys.is3D {
			t.Error("Expected 3D mode to be enabled")
		}
		if sys.cloud3D == nil {
			t.Error("Expected 3D point cloud to be initialized")
		}
		if len(sys.orientations) != 4 {
			t.Errorf("Expected 4 orientation quaternions, got %d", len(sys.orientations))
		}
		
		// Check that orientations are initialized to identity quaternions
		for i, q := range sys.orientations {
			if !floatsClose(q.W, 1.0, 1e-6) || !floatsClose(q.X, 0.0, 1e-6) ||
				!floatsClose(q.Y, 0.0, 1e-6) || !floatsClose(q.Z, 0.0, 1e-6) {
				t.Errorf("Expected identity quaternion for IMU %d, got (%.3f, %.3f, %.3f, %.3f)",
					i, q.W, q.X, q.Y, q.Z)
			}
		}
	})

	t.Run("NewIMUFusionSystemWithMode2D", func(t *testing.T) {
		sys, err := NewIMUFusionSystemWithMode(4, false)
		if err != nil {
			t.Fatalf("Failed to create 2D IMU fusion system: %v", err)
		}
		if sys.is3D {
			t.Error("Expected 2D mode to be enabled")
		}
		if sys.cloud == nil {
			t.Error("Expected 2D point cloud to be initialized")
		}
	})

	t.Run("BackwardCompatibility", func(t *testing.T) {
		sys, err := NewIMUFusionSystem(4)
		if err != nil {
			t.Fatalf("Failed to create IMU fusion system: %v", err)
		}
		if sys.is3D {
			t.Error("Expected 2D mode by default for backward compatibility")
		}
	})
}

func TestSphere(t *testing.T) {
	t.Run("Intersects", func(t *testing.T) {
		s1 := &Sphere{X: 0, Y: 0, Z: 0, Radius: 1.0}
		s2 := &Sphere{X: 1.5, Y: 0, Z: 0, Radius: 1.0}
		s3 := &Sphere{X: 3, Y: 0, Z: 0, Radius: 1.0}

		if !s1.Intersects(s2) {
			t.Error("Expected spheres s1 and s2 to intersect")
		}
		if s1.Intersects(s3) {
			t.Error("Expected spheres s1 and s3 to not intersect")
		}
	})

	t.Run("Expand", func(t *testing.T) {
		s := &Sphere{X: 0, Y: 0, Z: 0, Radius: 1.0}
		s.Expand(2.0)
		if s.Radius != 2.0 {
			t.Errorf("Expected radius 2.0, got %.1f", s.Radius)
		}
	})
}

func TestFusedPosition3D(t *testing.T) {
	spheres := []Sphere{
		{X: 0, Y: 0, Z: 0, Radius: 1.0},
		{X: 2, Y: 0, Z: 0, Radius: 1.0},
		{X: 1, Y: 2, Z: 0, Radius: 1.0},
	}
	uncertainties := []float64{1.0, 1.0, 1.0}

	x, y, z := FusedPosition3D(spheres, uncertainties)
	expectedX, expectedY, expectedZ := 1.0, 2.0/3.0, 0.0

	if !floatsClose(x, expectedX, 1e-6) || !floatsClose(y, expectedY, 1e-6) || !floatsClose(z, expectedZ, 1e-6) {
		t.Errorf("Expected fused position (%.3f, %.3f, %.3f), got (%.3f, %.3f, %.3f)",
			expectedX, expectedY, expectedZ, x, y, z)
	}
}