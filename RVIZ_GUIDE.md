# EKF Miniproject - RViz2 Visualization Guide

## Running the Demo

1. **Launch the EKF demo** (in terminal 1):
   ```bash
   cd ~/ekf_ws
   source install/setup.bash
   ros2 launch ekf_miniproject ekf_demo.launch.py
   ```

2. **Open RViz2** (in terminal 2):
   ```bash
   cd ~/ekf_ws
   source install/setup.bash
   rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
   ```
   
   Or use the helper script:
   ```bash
   ~/ekf_ws/open_rviz.sh
   ```

## RViz2 Display Configuration

The RViz configuration file includes the following displays:

### Displays Added:
1. **Grid** - Reference grid in the map frame
2. **RobotModel** - The four-wheel robot visualization
3. **EKF Path** (Green) - Shows the estimated path from the EKF
   - Topic: `/ekf/path`
   - Color: Green (25, 255, 0)
4. **Truth Path** (Red) - Shows the ground truth path
   - Topic: `/truth/path`
   - Color: Red (255, 0, 0)
5. **Truth Pose** (Red Arrow) - Current ground truth pose
   - Topic: `/truth/pose`
   - Shape: Red Arrow
6. **EKF Pose** (Green Arrow) - Current EKF estimated pose
   - Topic: `/ekf/pose`
   - Shape: Green Arrow

## Topics Being Published

The simulation publishes the following topics:

- `/cmd_vel` - Velocity commands (50 Hz)
- `/truth/pose` - Ground truth pose (50 Hz)
- `/truth/path` - Ground truth path (50 Hz)
- `/gps` - Noisy GPS measurements (5 Hz)
- `/imu_yaw` - Noisy IMU yaw measurements (50 Hz)
- `/ekf/pose` - EKF estimated pose (50 Hz)
- `/ekf/path` - EKF estimated path (50 Hz)

## Optional: Echo Topics

To see the raw topic data in the terminal:

**Terminal 3 - EKF Pose:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 topic echo /ekf/pose
```

**Terminal 4 - GPS:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 topic echo /gps
```

**Terminal 5 - Truth Pose:**
```bash
cd ~/ekf_ws
source install/setup.bash
ros2 topic echo /truth/pose
```

## What to Look For

In RViz2, you should see:
- **Green path**: EKF estimated trajectory (smoother)
- **Red path**: Ground truth trajectory
- The green path should closely follow the red path, showing that the EKF is successfully fusing GPS and IMU data
- The robot model will be positioned according to the robot state publisher

## Troubleshooting

If RViz2 shows nothing:
1. Check the Fixed Frame is set to `map`
2. Verify topics are being published: `ros2 topic list`
3. Check topic rates: `ros2 topic hz /ekf/path`
4. Make sure the launch file is still running

If RViz2 crashes on launch:
- This is a known library issue with some systems
- Use the manual launch command instead of auto-launching from the launch file
