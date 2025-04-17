package internal

import (
	"fmt"

	"gonum.org/v1/gonum/mat"
)

// Procrustes aligns two sets of points using least squares optimization.
// It returns the transformed source points, the target centroid, and the scale factor.
func Procrustes(source, target []Point) ([]Point, Point, float64) {
	// Calculate centroids of both point sets
	centroidSource := centroid(source)
	centroidTarget := centroid(target)

	// Center the points
	centeredSource := centerPoints(source, centroidSource)
	centeredTarget := centerPoints(target, centroidTarget)

	// Compute the covariance matrix H = X * Y^T
	H := computeCovarianceMatrix(centeredSource, centeredTarget)

	// Singular Value Decomposition (SVD) of H
	var svd mat.SVD
	svd.Factorize(H, mat.SVDThin)
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
		// Create a reflection matrix
		reflection := mat.NewDense(2, 2, []float64{1, 0, 0, -1})
		// Multiply V by the reflection matrix
		var Vcorrected mat.Dense
		Vcorrected.Mul(&V, reflection)
		// Recalculate R
		R.Mul(&Vcorrected, U.T())
		// Adjust the last singular value
		S[len(S)-1] = -S[len(S)-1]
	}

	// Calculate the optimal scale factor
	// scale = sum(S) / var(X)
	// where var(X) = sum(||centeredSource_i||^2)
	var sumS float64
	for _, val := range S {
		sumS += val
	}
	var varSource float64
	for _, p := range centeredSource {
		varSource += p.X*p.X + p.Y*p.Y
	}
	scale := 1.0 // Default scale
	if varSource != 0 {
		scale = sumS / varSource
	}

	// Convert R (gonum matrix) to [][]float64 for applyTransformation
	rotationMatrix := [][]float64{
		{R.At(0, 0), R.At(0, 1)},
		{R.At(1, 0), R.At(1, 1)},
	}

	// Apply the transformation (scale, rotation, translation)
	aligned := applyTransformation(centeredSource, scale, rotationMatrix, centroidTarget)

	return aligned, centroidTarget, scale
}

func centroid(points []Point) Point {
	var sumX, sumY float64
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
// where X is centeredSource and Y is centeredTarget.
func computeCovarianceMatrix(source, target []Point) *mat.Dense {
	n := len(source)
	if n == 0 || n != len(target) {
		// Return zero matrix or handle error appropriately
		return mat.NewDense(2, 2, nil)
	}

	// Create matrices for source (X) and target (Y) points
	// Each point is a column vector, so matrices are 2xN
	sourceData := make([]float64, 2*n)
	targetData := make([]float64, 2*n)
	for i := 0; i < n; i++ {
		sourceData[i] = source[i].X   // Row 0
		sourceData[i+n] = source[i].Y // Row 1
		targetData[i] = target[i].X   // Row 0
		targetData[i+n] = target[i].Y // Row 1
	}
	X := mat.NewDense(2, n, sourceData)
	Y := mat.NewDense(2, n, targetData)

	// Compute H = X * Y^T (Note: Gonum uses row-major, so X is effectively X^T conceptually here)
	// We want sum(source_i * target_i^T) which is X * Y^T if X, Y are 2xN
	var H mat.Dense
	H.Mul(X, Y.T()) // H is 2x2

	return &H
}

// applyTransformation applies scale, rotation, and translation to centered points
func applyTransformation(points []Point, scale float64, R [][]float64, translation Point) []Point {
	aligned := make([]Point, len(points))
	for i, p := range points {
		// Apply rotation first (to centered points p)
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
