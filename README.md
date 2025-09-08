# IMU Fusion System

> [!IMPORTANT]
> This project now supports both 2D and 3D implementations with full 6 degrees of freedom (6DOF) capability including orientation tracking using quaternions.

This project implements a robust and real-time Inertial Measurement Unit (IMU) fusion system designed to accurately estimate position and orientation using data from multiple IMUs. The system supports both 2D planar configurations and full 3D spatial arrangements with 6 degrees of freedom tracking.

The name. A tongue-in-cheek name derived from the fundamentals of the algorithm, which is a circle-based (2D) or sphere-based (3D) algorithm designed to reduce jitter (jerking) in IMU timeseries data.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Overview

The IMU Fusion System aims to achieve high accuracy in position and orientation estimation while maintaining real-time performance. The system supports:

- **2D Mode**: Traditional planar IMU fusion for applications constrained to 2D movement
- **3D Mode**: Full 3D spatial tracking with 6 degrees of freedom (3 translational + 3 rotational)
- **Quaternion-based Orientation**: Robust orientation tracking using quaternions to avoid gimbal lock
- **Multi-IMU Fusion**: Processes data from multiple IMUs at frame rates exceeding 1000Hz
- **Real-time Performance**: Low computational overhead and robustness against measurement uncertainties

## Architecture

The system operates in the following stages:

### 2D Mode
1. **IMU Data Acquisition**: Collects acceleration and angular velocity data from multiple IMUs and synchronizes the data temporally.
2. **Individual Position Estimation**: Integrates acceleration to compute 2D position estimates for each IMU and estimates uncertainty based on noise and integration drift.
3. **Geometric Fusion**: Models each position estimate as a circle and computes an initial fused estimate while applying rigid body transformations to enforce fixed distances.
4. **Point Cloud Generation**: Maps real-time IMU position samples into a 2D point cloud.
5. **Position Refinement**: Projects the fused position onto the point cloud using nearest neighbor search or mean of nearby points.

### 3D Mode with 6DOF
1. **IMU Data Acquisition**: Collects 3D acceleration and angular velocity data from multiple IMUs with temporal synchronization.
2. **Position and Orientation Integration**: 
   - Integrates 3D acceleration to compute position estimates
   - Integrates angular velocity using quaternion mathematics for orientation tracking
3. **3D Geometric Fusion**: Models each position estimate as a sphere and computes fused estimates using 3D sphere intersection algorithms.
4. **3D Point Cloud Generation**: Maps real-time IMU position samples into a 3D point cloud with spatial indexing.
5. **6DOF State Estimation**: Outputs both position (X, Y, Z) and orientation (quaternion) for full 6DOF tracking.
6. **Position Refinement**: Projects the fused position onto the 3D point cloud for drift correction.

## Installation

To install the IMU Fusion System, ensure you have Go installed on your machine. Clone the repository and navigate to the project directory:

```bash
git clone <repository-url>
cd imu-fusion
```

Then, run the following command to download the necessary dependencies:

```bash
go mod tidy
```

## Usage

To run the IMU Fusion System, execute the following command:

```bash
go run cmd/main.go
```

This will initialize both 2D and 3D systems simultaneously, demonstrating:
- **2D IMU Fusion**: Traditional planar tracking with circle-based uncertainty regions
- **3D IMU Fusion with 6DOF**: Full spatial tracking with sphere-based uncertainty regions and quaternion-based orientation

### Programmatic Usage

#### 2D Mode (Backward Compatible)
```go
// Create a 2D IMU fusion system (default mode)
imuSystem, err := internal.NewIMUFusionSystem(4)
if err != nil {
    log.Fatalf("Failed to initialize IMU fusion system: %v", err)
}
imuSystem.Start()
```

#### 3D Mode with 6DOF
```go
// Create a 3D IMU fusion system with 6DOF tracking
imuSystem3D, err := internal.NewIMUFusionSystemWithMode(4, true)
if err != nil {
    log.Fatalf("Failed to initialize 3D IMU fusion system: %v", err)
}
imuSystem3D.Start()
```

### Features Demonstrated
- Real-time position fusion in both 2D and 3D
- Quaternion-based orientation tracking (3D mode)
- Uncertainty quantification with geometric fusion
- Point cloud-based drift correction
- Multi-IMU data synchronization

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
