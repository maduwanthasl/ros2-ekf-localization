#!/bin/bash
# Try multiple methods to launch RViz2

cd ~/ekf_ws
source install/setup.bash

echo "Attempting to launch RViz2..."
echo ""

# Method 1: Try with LD_PRELOAD
echo "Method 1: Using LD_PRELOAD..."
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz 2>&1 &
RVIZ_PID=$!

# Wait a moment to see if it crashes
sleep 2

# Check if still running
if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "✓ RViz2 launched successfully!"
    wait $RVIZ_PID
else
    echo "✗ Method 1 failed, trying Method 2..."
    echo ""
    
    # Method 2: Try without any library manipulation
    echo "Method 2: Standard launch..."
    rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz 2>&1
fi
