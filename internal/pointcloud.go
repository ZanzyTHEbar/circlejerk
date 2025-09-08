package internal

import (
	"sync"

	"github.com/kyroy/kdtree"
)

// Point implements kdtree.Point interface for 2D
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

// Point3D implements kdtree.Point interface for 3D
func (p Point3D) Dimensions() int { return 3 }
func (p Point3D) Dimension(i int) float64 {
	switch i {
	case 0:
		return p.X
	case 1:
		return p.Y
	case 2:
		return p.Z
	default:
		return 0
	}
}
func (p Point3D) Distance(q kdtree.Point) float64 {
	qp := q.(Point3D)
	dx := p.X - qp.X
	dy := p.Y - qp.Y
	dz := p.Z - qp.Z
	return dx*dx + dy*dy + dz*dz
}

// PointCloud with k-d tree support for 2D points
type PointCloud struct {
	points []Point
	tree   *kdtree.KDTree
	mu     sync.Mutex
}

// PointCloud3D with k-d tree support for 3D points
type PointCloud3D struct {
	points []Point3D
	tree   *kdtree.KDTree
	mu     sync.Mutex
}

// NewPointCloud initializes a new 2D PointCloud.
func NewPointCloud() *PointCloud {
	return &PointCloud{
		points: make([]Point, 0),
		tree:   nil,
	}
}

// NewPointCloud3D initializes a new 3D PointCloud.
func NewPointCloud3D() *PointCloud3D {
	return &PointCloud3D{
		points: make([]Point3D, 0),
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

// Helper to convert []Point3D to []kdtree.Point
func points3DToKDTreePoints(points []Point3D) []kdtree.Point {
	out := make([]kdtree.Point, len(points))
	for i, p := range points {
		out[i] = p
	}
	return out
}

// AddPoint adds a new point to the 2D point cloud and updates the k-d tree.
func (pc *PointCloud) AddPoint(x, y float64) {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pt := Point{X: x, Y: y}
	pc.points = append(pc.points, pt)
	// Convert to []kdtree.Point for tree
	pc.tree = kdtree.New(pointsToKDTreePoints(pc.points))
}

// AddPoint adds a new point to the 3D point cloud and updates the k-d tree.
func (pc *PointCloud3D) AddPoint(x, y, z float64) {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pt := Point3D{X: x, Y: y, Z: z}
	pc.points = append(pc.points, pt)
	// Convert to []kdtree.Point for tree
	pc.tree = kdtree.New(points3DToKDTreePoints(pc.points))
}

// GetPoints returns a copy of the 2D points in the point cloud.
func (pc *PointCloud) GetPoints() []Point {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pointsCopy := make([]Point, len(pc.points))
	copy(pointsCopy, pc.points)
	return pointsCopy
}

// GetPoints returns a copy of the 3D points in the point cloud.
func (pc *PointCloud3D) GetPoints() []Point3D {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pointsCopy := make([]Point3D, len(pc.points))
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

// RadiusSearch returns all 2D points within radius of (x, y) using a linear scan.
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

// RadiusSearch returns all 3D points within radius of (x, y, z) using a linear scan.
func (pc *PointCloud3D) RadiusSearch(x, y, z, radius float64) []Point3D {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	var result []Point3D
	r2 := radius * radius
	for _, pt := range pc.points {
		dx := pt.X - x
		dy := pt.Y - y
		dz := pt.Z - z
		if dx*dx+dy*dy+dz*dz <= r2 {
			result = append(result, pt)
		}
	}
	return result
}

// Clear clears the 2D point cloud and k-d tree.
func (pc *PointCloud) Clear() {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.points = make([]Point, 0)
	pc.tree = nil
}

// Clear clears the 3D point cloud and k-d tree.
func (pc *PointCloud3D) Clear() {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.points = make([]Point3D, 0)
	pc.tree = nil
}
