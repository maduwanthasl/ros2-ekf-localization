# ROS2 Extended Kalman Filter for Mobile Robot Localization

<img width="630" height="440" alt="image" src="https://github.com/user-attachments/assets/4fad228d-4030-4f5c-a84f-2ae2c2b12273" />

*Image source: https://doi.org/10.1080/21642583.2013.864249*

A complete ROS2 implementation of an Extended Kalman Filter (EKF) for 2D mobile robot localization through GPS and IMU sensor fusion. This project demonstrates state estimation techniques with real-time visualization and comprehensive performance analysis tools.

## Features

- **Extended Kalman Filter**: 3-state EKF (x, y, yaw) implementation from scratch
- **Sensor Fusion**: Combines GPS (5Hz) and IMU (50Hz) measurements for robust localization
- **Robot Simulator**: Unicycle model with configurable Gaussian noise for realistic sensor data
- **RViz2 Visualization**: Real-time visualization of ground truth vs. estimated trajectories
- **ROS2 Jazzy**: Built with modern ROS2 architecture using Python 3.12

## Key Results

- Position RMSE: ~10 cm with 10 cm GPS noise
- 100+ Hz estimation from 5 Hz GPS measurements
- Demonstrates significant improvement over dead-reckoning

---

## Simulation Details

### Robot Model
- **Kinematics**: Unicycle model with differential drive
- **Dynamics**: Controlled circular trajectory
  - Linear velocity: 0.4 m/s
  - Angular velocity: 0.6 rad/s
  - Trajectory radius: ~0.67 m
- **Update Rate**: 50 Hz

### Sensor Characteristics
- **GPS**:
  - Measurement rate: 5 Hz
  - Noise: Gaussian with σ = 0.10 m (both x and y)
  - Provides absolute position measurements
  
- **IMU (Yaw)**:
  - Measurement rate: 50 Hz
  - Noise: Gaussian with σ = 0.03 rad
  - Provides heading angle measurements

### EKF Configuration
- **State Vector**: [x, y, yaw]ᵀ
- **Process Model**: Unicycle kinematics with motion integration
- **Measurement Models**: 
  - GPS: Direct position observation
  - IMU: Direct yaw observation
- **Covariance Tuning**: Optimized for 10 cm position accuracy

---

## Results and Analysis

### 1. Trajectory Comparison

![2D Trajectory](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/trajectory_2d.png)
The trajectory plot shows:

- **Ground Truth** (black solid line): The actual robot path.
- **EKF Estimate** (blue solid line): The filtered state estimate closely tracking the ground truth.
- **GPS Measurements** (orange crosses): Noisy position measurements with approximately ±10 cm standard deviation.
- **Start** (green marker) and **End** (red marker): Initial and final robot positions.

**Key Observation**: The EKF estimate remains tightly bound to the ground truth despite significant GPS noise, demonstrating effective sensor fusion and error correction. This highlights the advantage of EKF-based localization over raw GPS measurements.
### 2. Position Error Analysis

![Position Error vs Time](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/position_error_vs_time.png)

This plot demonstrates:
- **X-axis error** (top): Mean error 6.7 cm, max ~15 cm
- **Y-axis error** (bottom): Mean error 7.6 cm, max ~18 cm
- **Overall RMSE**: 10.13 cm

The errors remain bounded and consistent throughout the simulation, showing the EKF successfully filters out GPS noise while maintaining accurate state estimates.

![Combined Position Error](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/position_error_combined.png)

Combined view shows both X and Y errors oscillate around zero with consistent magnitude, indicating no systematic bias in either direction.

### 3. Heading Error

![Yaw Error vs Time](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/yaw_error_vs_time.png)

Heading estimation maintains:
- **Mean error**: <0.05 radians (~3 degrees)
- **Stability**: Bounded errors throughout circular trajectory
- **Convergence**: Quick settling from initial conditions

### 4. Sensor Fusion Performance

![GPS vs EKF](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/gps_vs_ekf.png)

This overlay demonstrates:
- **GPS measurements** (scattered points): Show significant noise in both X and Y
- **EKF estimates** (smooth lines): Successfully filter noise while tracking the true trajectory
- **Innovation statistics**: GPS noise std ~10 cm matches configured sensor noise

![IMU vs EKF](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/imu_vs_ekf.png)

IMU yaw fusion shows:
- **Smooth yaw estimation** from noisy IMU measurements
- **Consistent angular velocity** during circular motion
- **Effective noise rejection** while preserving dynamics

### 5. EKF vs Dead-Reckoning Comparison

![DR vs EKF Error](https://github.com/maduwanthasl/ros2-ekf-localization/blob/main/figures/dr_vs_ekf_error.png)

Performance comparison reveals:
- **Dead-Reckoning Error**: Grows unbounded due to integration drift
- **EKF Error**: Remains bounded at ~10 cm
- **Improvement**: >90% error reduction through sensor fusion
- **Key Benefit**: EKF provides long-term stability that dead-reckoning cannot achieve

**Conclusion**: The Extended Kalman Filter successfully fuses noisy GPS and IMU measurements to achieve 10 cm position accuracy—matching the GPS sensor noise level—while operating at 100+ Hz, demonstrating effective sensor fusion and state estimation.
