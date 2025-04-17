package internal

import (
	"sync"

	"github.com/kyroy/kdtree"
)

// Point implements kdtree.Point interface
func (p Point) Dimensions() int { return 2 }
func (p Point) Dimension(i int) float64 {
	if i == 0 {
		return p.X
	}
	return p.Y
}
func (p Point) Distance(q kdtree.Point) float64 {
	qp := q.(Point)
	dx := p.X - qp.X
	dy := p.Y - qp.Y
	return dx*dx + dy*dy
}

// PointCloud with k-d tree support
type PointCloud struct {
	points []Point
	tree   *kdtree.KDTree
	mu     sync.Mutex
}

// NewPointCloud initializes a new PointCloud.
func NewPointCloud() *PointCloud {
	return &PointCloud{
		points: make([]Point, 0),
		tree:   nil,
	}
}

// Helper to convert []Point to []kdtree.Point
func pointsToKDTreePoints(points []Point) []kdtree.Point {
	out := make([]kdtree.Point, len(points))
	for i, p := range points {
		out[i] = p
	}
	return out
}

// AddPoint adds a new point to the point cloud and updates the k-d tree.
func (pc *PointCloud) AddPoint(x, y float64) {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pt := Point{X: x, Y: y}
	pc.points = append(pc.points, pt)
	// Convert to []kdtree.Point for tree
	pc.tree = kdtree.New(pointsToKDTreePoints(pc.points))
}

// GetPoints returns a copy of the points in the point cloud.
func (pc *PointCloud) GetPoints() []Point {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pointsCopy := make([]Point, len(pc.points))
	copy(pointsCopy, pc.points)
	return pointsCopy
}

// TODO: radiusSearcher implements kdtree.Searcher for radius queries
// (see https://pkg.go.dev/github.com/kyroy/kdtree#section-readme)
type radiusSearcher struct {
	query  Point
	radius float64
	found  []Point
}

func (s *radiusSearcher) Search(point kdtree.Point, dist float64) (done bool) {
	if dist <= s.radius*s.radius {
		s.found = append(s.found, point.(Point))
	}
	return false // keep searching
}

// RadiusSearch returns all points within radius of (x, y) using a linear scan.
func (pc *PointCloud) RadiusSearch(x, y, radius float64) []Point {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	var result []Point
	r2 := radius * radius
	for _, pt := range pc.points {
		dx := pt.X - x
		dy := pt.Y - y
		if dx*dx+dy*dy <= r2 {
			result = append(result, pt)
		}
	}
	return result
}

// Clear clears the point cloud and k-d tree.
func (pc *PointCloud) Clear() {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.points = make([]Point, 0)
	pc.tree = nil
}
