package internal

import "sync"

// PointCloud stores points for local refinement.
type PointCloud struct {
	points []Point
	mu     sync.Mutex
}

// NewPointCloud initializes a new PointCloud.
func NewPointCloud() *PointCloud {
	return &PointCloud{
		points: make([]Point, 0),
	}
}

// AddPoint adds a new point to the point cloud.
func (pc *PointCloud) AddPoint(x, y float64) {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.points = append(pc.points, Point{X: x, Y: y})
}

// GetPoints returns a copy of the points in the point cloud.
func (pc *PointCloud) GetPoints() []Point {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pointsCopy := make([]Point, len(pc.points))
	copy(pointsCopy, pc.points)
	return pointsCopy
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

// Clear clears the point cloud.
func (pc *PointCloud) Clear() {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.points = make([]Point, 0)
}
