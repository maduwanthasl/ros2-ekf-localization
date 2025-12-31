#!/bin/bash
# Record EKF data for analysis
# Records truth, EKF estimates, GPS, and IMU data

cd ~/ekf_ws
source install/setup.bash

echo "Recording rosbag for 60 seconds..."
echo "Press Ctrl+C to stop early"
echo ""
echo "Topics being recorded:"
echo "  - /truth/pose (ground truth)"
echo "  - /ekf/pose (EKF estimate)"
echo "  - /gps (noisy GPS)"
echo "  - /imu_yaw (noisy IMU)"
echo "  - /cmd_vel (velocity commands)"
echo ""

# Create bags directory if it doesn't exist
mkdir -p bags

# Record the bag
ros2 bag record /truth/pose /ekf/pose /gps /imu_yaw /cmd_vel -o bags/ekf_bag

echo ""
echo "Recording complete! Bag saved to bags/ekf_bag"
echo "To plot the results, run: python3 plot_ekf_results.py"
