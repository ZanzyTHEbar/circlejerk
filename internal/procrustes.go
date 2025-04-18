package internal

import (
	"fmt"

	"gonum.org/v1/gonum/mat"
)

// Procrustes aligns two sets of points using least squares optimization.
// It returns the transformed source points, the target centroid, and the scale factor.
func Procrustes(source, target []Point) ([]Point, Point, float64) {
	if len(source) == 0 || len(target) == 0 || len(source) != len(target) {
		// Handle cases with empty or mismatched input sizes
		// Returning empty results or an error might be appropriate
		fmt.Println("Procrustes: Warning - empty or mismatched input point sets.")
		return []Point{}, Point{}, 0.0
	}
	if len(source) < 2 {
		// Procrustes requires at least 2 points for meaningful alignment (rotation/scale)
		// If only 1 point, can only translate. Return translated source point.
		fmt.Println("Procrustes: Warning - only one point provided. Performing translation only.")
		centroidSource := centroid(source)
		centroidTarget := centroid(target)
		translation := Point{X: centroidTarget.X - centroidSource.X, Y: centroidTarget.Y - centroidSource.Y}
		aligned := []Point{{X: source[0].X + translation.X, Y: source[0].Y + translation.Y}}
		return aligned, centroidTarget, 1.0 // Scale is undefined/irrelevant, return 1.0
	}

	// Calculate centroids of both point sets
	centroidSource := centroid(source)
	centroidTarget := centroid(target)

	// Center the points
	centeredSource := centerPoints(source, centroidSource)
	centeredTarget := centerPoints(target, centroidTarget)

	// Compute the covariance matrix H = X * Y^T
	H := computeCovarianceMatrix(centeredSource, centeredTarget)
	if H == nil { // Check if computeCovarianceMatrix returned nil (error case)
		fmt.Println("Procrustes: Error computing covariance matrix.")
		return []Point{}, Point{}, 0.0
	}

	// Singular Value Decomposition (SVD) of H
	var svd mat.SVD
	ok := svd.Factorize(H, mat.SVDThin)
	if !ok {
		fmt.Println("Procrustes: SVD factorization failed.")
		return []Point{}, Point{}, 0.0 // Or handle error appropriately
	}
	var U, V mat.Dense
	svd.UTo(&U)
	svd.VTo(&V)
	S := svd.Values(nil) // Singular values

	// Compute the rotation matrix R = V * U^T
	var R mat.Dense
	R.Mul(&V, U.T())

	// Handle potential reflection case (det(R) == -1)
	if mat.Det(&R) < 0 {
		fmt.Println("Procrustes: Reflection detected, correcting rotation matrix.")
		// Create a reflection matrix for 2D
		// reflection := mat.NewDense(2, 2, []float64{1, 0, 0, -1}) // This reflects across the x-axis
		// To correct R = V * U^T when det(R) = -1, we multiply the column of V
		// corresponding to the smallest singular value by -1.
		// Assuming SVD returns singular values in descending order, the last one is smallest.
		Vcols := V.RawMatrix().Cols
		Vrows := V.RawMatrix().Rows
		Vdata := V.RawMatrix().Data
		correctedVData := make([]float64, len(Vdata))
		copy(correctedVData, Vdata)
		// Modify the last column of V
		for r := 0; r < Vrows; r++ {
			correctedVData[r*Vcols+(Vcols-1)] *= -1
		}
		Vcorrected := mat.NewDense(Vrows, Vcols, correctedVData)

		// Recalculate R
		R.Mul(Vcorrected, U.T())
		// Adjust the last singular value conceptually (though not used directly in R calculation after correction)
		// S[len(S)-1] = -S[len(S)-1] // This modification to S is mainly for scale calculation consistency if needed
	}

	// Calculate the optimal scale factor
	// scale = sum(trace(S @ D)) / var(X) where D handles reflection (already done by correcting R)
	// scale = trace(R^T * Y^T * X) / trace(X^T * X) = trace(H * R) / var(X)
	// Simplified scale = sum(S) / var(X) if no reflection, or sum(diag(D)*S) / var(X) with reflection correction D.
	// Since we corrected R, we can use sum(S) where S are the original singular values.
	var sumS float64
	for _, val := range S {
		sumS += val
	}
	var varSource float64
	for _, p := range centeredSource {
		varSource += p.X*p.X + p.Y*p.Y // sum(||centeredSource_i||^2)
	}

	scale := 1.0 // Default scale
	if varSource > epsilon { // Avoid division by zero or near-zero
		// If reflection was detected and corrected in R, the scale should use the corrected singular values conceptually.
		// However, the standard approach often uses sum(S) directly after ensuring det(R)=1.
		// Let's stick to sum(S) / varSource after R correction.
		scale = sumS / varSource
	} else {
		fmt.Println("Procrustes: Warning - Variance of source points is near zero. Scale calculation might be unstable.")
		// If variance is zero, points are likely coincident. Scale is ambiguous.
		// Keep scale = 1.0 or handle as an error/special case.
	}

	// Convert R (gonum matrix) to [][]float64 for applyTransformation
	// Ensure R is 2x2
	rRows, rCols := R.Dims()
	if rRows != 2 || rCols != 2 {
		fmt.Println("Procrustes: Error - Rotation matrix is not 2x2.")
		return []Point{}, Point{}, 0.0
	}
	rotationMatrix := [][]float64{
		{R.At(0, 0), R.At(0, 1)},
		{R.At(1, 0), R.At(1, 1)},
	}

	// Apply the transformation (scale, rotation, translation)
	// Transformation: p' = scale * R * p_centered + centroidTarget
	aligned := applyTransformation(centeredSource, scale, rotationMatrix, centroidTarget)

	return aligned, centroidTarget, scale
}

func centroid(points []Point) Point {
	var sumX, sumY float64
	if len(points) == 0 {
		return Point{} // Return zero point if no points
	}
	for _, p := range points {
		sumX += p.X
		sumY += p.Y
	}
	n := float64(len(points))
	return Point{X: sumX / n, Y: sumY / n}
}

func centerPoints(points []Point, centroid Point) []Point {
	centered := make([]Point, len(points))
	for i, p := range points {
		centered[i] = Point{X: p.X - centroid.X, Y: p.Y - centroid.Y}
	}
	return centered
}

// computeCovarianceMatrix computes the covariance matrix H = X * Y^T
// where X is centeredSource (2xN) and Y is centeredTarget (2xN).
func computeCovarianceMatrix(source, target []Point) *mat.Dense {
	n := len(source)
	// Basic validation already done in Procrustes, but double-check here
	if n == 0 || n != len(target) {
		fmt.Println("computeCovarianceMatrix: Error - Empty or mismatched input.")
		return nil // Return nil to indicate error
	}

	// Create matrices for source (X) and target (Y) points
	// Each point is a column vector, so matrices are 2xN
	sourceData := make([]float64, 2*n)
	targetData := make([]float64, 2*n)
	for i := 0; i < n; i++ {
		sourceData[i] = source[i].X   // Row 0, Col i
		sourceData[i+n] = source[i].Y // Row 1, Col i
		targetData[i] = target[i].X   // Row 0, Col i
		targetData[i+n] = target[i].Y // Row 1, Col i
	}
	// X and Y are Dense matrices with dimensions 2 rows, n columns
	X := mat.NewDense(2, n, sourceData)
	Y := mat.NewDense(2, n, targetData)

	// Compute H = X * Y^T
	// Y is 2xN, Y.T() is Nx2
	// X is 2xN
	// H = X (2xN) * Y.T() (Nx2) results in a 2x2 matrix
	var H mat.Dense
	H.Mul(X, Y.T())

	return &H
}

// applyTransformation applies scale, rotation, and translation to centered points
// p_aligned = scale * R * p_centered + translation (centroidTarget)
func applyTransformation(centeredPoints []Point, scale float64, R [][]float64, translation Point) []Point {
	aligned := make([]Point, len(centeredPoints))
	// Ensure R is 2x2
	if len(R) != 2 || len(R[0]) != 2 || len(R[1]) != 2 {
		fmt.Println("applyTransformation: Error - Rotation matrix must be 2x2.")
		// Return original centered points or handle error
		copy(aligned, centeredPoints) // Or return nil/error
		return aligned // Return something to avoid panic, maybe untransformed points
	}

	for i, p := range centeredPoints {
		// Apply rotation first (to centered points p)
		// rotated = R * p
		rotatedX := R[0][0]*p.X + R[0][1]*p.Y
		rotatedY := R[1][0]*p.X + R[1][1]*p.Y
		// Apply scale
		scaledX := scale * rotatedX
		scaledY := scale * rotatedY
		// Then apply translation (target centroid)
		aligned[i] = Point{X: scaledX + translation.X, Y: scaledY + translation.Y}
	}
	return aligned
}
