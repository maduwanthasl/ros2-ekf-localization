#!/bin/bash
# Alternative RViz2 launcher with library preload to avoid snap conflicts

cd ~/ekf_ws
source install/setup.bash

# Method 1: Use system libraries, block snap
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 \
  rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz 2>&1

# If that fails, the script will exit and you can try the manual methods in USAGE_GUIDE.md
