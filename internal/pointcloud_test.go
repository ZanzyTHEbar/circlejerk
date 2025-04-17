package internal

import (
	"sort"
	"testing"
)

// Helper to compare slices of Points (ignoring order)
func pointSlicesEqual(a, b []Point, tol float64) bool {
	if len(a) != len(b) {
		return false
	}
	// Sort both slices based on X, then Y for consistent comparison
	sort.Slice(a, func(i, j int) bool {
		if a[i].X != a[j].X {
			return a[i].X < a[j].X
		}
		return a[i].Y < a[j].Y
	})
	sort.Slice(b, func(i, j int) bool {
		if b[i].X != b[j].X {
			return b[i].X < b[j].X
		}
		return b[i].Y < b[j].Y
	})

	for i := range a {
		if !pointsClose(a[i], b[i], tol) { // Use pointsClose from procrustes_test.go (or redefine)
			return false
		}
	}
	return true
}

// Redefine pointsClose if not accessible across test files
// func pointsClose(p1, p2 Point, tol float64) bool {
// 	return math.Abs(p1.X-p2.X) < tol && math.Abs(p1.Y-p2.Y) < tol
// }

func TestPointCloud_AddAndGet(t *testing.T) {
	pc := NewPointCloud()
	pointsToAdd := []Point{
		{1, 2},
		{3, 4},
		{-1, 0},
	}

	for _, p := range pointsToAdd {
		pc.AddPoint(p.X, p.Y)
	}

	retrievedPoints := pc.GetPoints()

	if !pointSlicesEqual(retrievedPoints, pointsToAdd, 1e-9) {
		t.Errorf("Expected points %v, got %v", pointsToAdd, retrievedPoints)
	}

	// Test adding more points
	morePoints := []Point{{5, 5}}
	pc.AddPoint(morePoints[0].X, morePoints[0].Y)
	allPoints := append(pointsToAdd, morePoints...)
	retrievedPoints = pc.GetPoints()

	if !pointSlicesEqual(retrievedPoints, allPoints, 1e-9) {
		t.Errorf("Expected points %v after adding more, got %v", allPoints, retrievedPoints)
	}
}

func TestPointCloud_RadiusSearch(t *testing.T) {
	pc := NewPointCloud()
	points := []Point{
		{0, 0},
		{1, 0},
		{0, 1},
		{1, 1},   // Inside radius 1.5 from (0,0)
		{2, 0},   // Outside radius 1.5
		{0, 2},   // Outside radius 1.5
		{-1, -1}, // Inside radius 1.5
	}
	for _, p := range points {
		pc.AddPoint(p.X, p.Y)
	}

	searchX, searchY := 0.0, 0.0
	radius := 1.5
	expected := []Point{
		{0, 0},
		{1, 0},
		{0, 1},
		{1, 1},
		{-1, -1},
	}

	found := pc.RadiusSearch(searchX, searchY, radius)

	if !pointSlicesEqual(found, expected, 1e-9) {
		t.Errorf("RadiusSearch(%f, %f, %f): Expected %v, got %v", searchX, searchY, radius, expected, found)
	}

	// Test search with no results
	foundEmpty := pc.RadiusSearch(10, 10, 1)
	if len(foundEmpty) != 0 {
		t.Errorf("Expected empty result for search far away, got %v", foundEmpty)
	}
}

func TestPointCloud_Clear(t *testing.T) {
	pc := NewPointCloud()
	pc.AddPoint(1, 1)
	pc.AddPoint(2, 2)

	if len(pc.GetPoints()) == 0 {
		t.Fatal("PointCloud should have points before Clear()")
	}

	pc.Clear()

	if len(pc.GetPoints()) != 0 {
		t.Errorf("Expected PointCloud to be empty after Clear(), got %d points", len(pc.GetPoints()))
	}

	// Ensure tree is also cleared (implicitly tested by AddPoint rebuilding it)
	// Add a point after clearing to ensure it works
	pc.AddPoint(3, 3)
	if len(pc.GetPoints()) != 1 {
		t.Errorf("Expected 1 point after adding post-Clear(), got %d", len(pc.GetPoints()))
	}
}
