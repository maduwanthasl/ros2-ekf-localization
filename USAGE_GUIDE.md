# EKF Project - Complete Usage Guide

## Quick Start

### 1. Launch the EKF Demo

**Terminal 1:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 launch ekf_miniproject ekf_demo.launch.py
```

This starts:
- Robot State Publisher
- Simulator (publishes truth, GPS, IMU)
- EKF Node (fuses sensor data)

### 2. Open RViz2 for Visualization

**Terminal 2:**
```bash
cd ~/ekf_ws
./open_rviz.sh
```

**If RViz2 crashes with library errors, try:**
```bash
cd ~/ekf_ws
source install/setup.bash
unset GTK_PATH
rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
```

**What you'll see in RViz2:**
- **Red path/arrow**: Ground truth trajectory and pose
- **Green path/arrow**: EKF estimated trajectory and pose
- **Blue dots**: Noisy GPS measurements
- **Robot model**: Four-wheel robot

### 3. Monitor Topics (Optional)

**Terminal 3 - EKF Pose:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 topic echo /ekf/pose
```

**Terminal 4 - GPS Data:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 topic echo /gps
```

**Terminal 5 - List All Topics:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 topic list
```

## Recording Data for Analysis

### Record a Bag File

**Terminal 2 (while demo is running):**
```bash
cd ~/ekf_ws
./record_bag.sh
```

This records for 60 seconds (or until Ctrl+C):
- `/truth/pose` - Ground truth
- `/ekf/pose` - EKF estimates
- `/gps` - GPS measurements
- `/imu_yaw` - IMU measurements

Bag is saved to: `bags/ekf_bag/`

### Manual Recording (alternative):
```bash
cd ~/ekf_ws
source install/setup.bash
mkdir -p bags
ros2 bag record /truth/pose /ekf/pose /gps /imu_yaw -o bags/ekf_bag
```

## Plotting Results

### Install Required Package

```bash
pip install rosbags matplotlib numpy
```

### Generate Plots

```bash
cd ~/ekf_ws
python3 plot_ekf_results.py bags/ekf_bag
```

**This creates:**
1. **XY Trajectory Plot**: Truth (red) vs EKF (green) vs GPS (blue scatter)
2. **X Error Plot**: Error over time with ±2σ bounds
3. **Y Error Plot**: Error over time with ±2σ bounds
4. **Yaw Error Plot**: Angular error over time with ±2σ bounds
5. **Position Error Magnitude**: Combined XY error
6. **Statistics Panel**: RMSE, mean, std for all errors

**Output:**
- Interactive plot window
- Saved image: `ekf_results.png`

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/truth/pose` | PoseStamped | 50 Hz | Ground truth position |
| `/truth/path` | Path | 50 Hz | Ground truth trajectory |
| `/ekf/pose` | PoseStamped | 50 Hz | EKF estimated position |
| `/ekf/path` | Path | 50 Hz | EKF estimated trajectory |
| `/gps` | PoseStamped | 5 Hz | Noisy GPS measurements |
| `/imu_yaw` | Float64 | 50 Hz | Noisy IMU yaw |
| `/cmd_vel` | Twist | 50 Hz | Velocity commands |

## Project Structure

```
~/ekf_ws/
├── src/ekf_miniproject/
│   ├── ekf_miniproject/
│   │   ├── ekf_node.py          # EKF implementation
│   │   └── sim_node.py          # Simulator
│   ├── launch/
│   │   └── ekf_demo.launch.py  # Launch file
│   ├── rviz/
│   │   └── ekf_demo.rviz       # RViz config
│   └── urdf/
│       └── four_wheel_bot.urdf.xacro
├── open_rviz.sh                # RViz launcher
├── record_bag.sh               # Bag recording script
├── plot_ekf_results.py         # Plotting script
├── bags/                       # Recorded bag files
└── USAGE_GUIDE.md             # This file
```

## Troubleshooting

### RViz2 Library Error
If you get `symbol lookup error: libpthread.so.0`:
1. The modified `open_rviz.sh` script filters out snap libraries
2. Alternatively, try: `unset LD_LIBRARY_PATH && rviz2`
3. Or manually open RViz and load the config file

### No Topics Visible
1. Check launch file is running: `ps aux | grep ros`
2. List topics: `ros2 topic list`
3. Check topic rates: `ros2 topic hz /ekf/pose`

### Plotting Fails
1. Install rosbags: `pip install rosbags matplotlib numpy`
2. Check bag exists: `ls -la bags/ekf_bag/`
3. Verify bag has data: `ros2 bag info bags/ekf_bag`

### Build Errors
```bash
cd ~/ekf_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## For Your Report

The plotting script generates publication-ready figures showing:
- **Trajectory comparison**: Visual proof EKF tracks truth
- **Error plots**: Quantitative performance metrics
- **Uncertainty bounds**: ±2σ showing filter consistency
- **Statistics**: RMSE, mean, std for all error metrics

Save `ekf_results.png` for your report!

## Commands Cheat Sheet

```bash
# Build
cd ~/ekf_ws && colcon build && source install/setup.bash

# Launch
ros2 launch ekf_miniproject ekf_demo.launch.py

# Visualize
./open_rviz.sh

# Record
./record_bag.sh

# Plot
python3 plot_ekf_results.py bags/ekf_bag

# Check topics
ros2 topic list
ros2 topic hz /ekf/pose
ros2 topic echo /ekf/pose
```
