# IMU Fusion System

This project implements a robust and real-time Inertial Measurement Unit (IMU) fusion system designed to accurately estimate position using data from multiple IMUs arranged in a planar configuration. The system leverages geometric constraints and a real-time 2D point cloud to mitigate drift and noise inherent in IMU measurements.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Overview

The IMU Fusion System aims to achieve high accuracy in position estimation while maintaining real-time performance. The system processes data from four IMUs at a frame rate exceeding 1000Hz, ensuring low computational overhead and robustness against measurement uncertainties.

## Architecture

The system operates in the following stages:

1. **IMU Data Acquisition**: Collects acceleration and angular velocity data from four IMUs and synchronizes the data temporally.
2. **Individual Position Estimation**: Integrates acceleration and angular velocity to compute position estimates for each IMU and estimates uncertainty based on noise and integration drift.
3. **Geometric Fusion**: Models each position estimate as a circle and computes an initial fused estimate while applying rigid body transformations to enforce fixed distances.
4. **Point Cloud Generation**: Maps real-time IMU position samples into a 2D point cloud.
5. **Position Refinement**: Projects the fused position onto the point cloud using nearest neighbor search or mean of nearby points.

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

This will initialize the system, set up data acquisition, and start processing the IMU data.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.