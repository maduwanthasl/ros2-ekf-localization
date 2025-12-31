#!/bin/bash
# RViz2 launcher with library conflict workaround

cd ~/ekf_ws
source install/setup.bash

echo "Launching RViz2..."

# Try to use system pthread library instead of snap's
if [ -f /lib/x86_64-linux-gnu/libpthread.so.0 ]; then
    echo "Using LD_PRELOAD workaround..."
    LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
elif [ -f /usr/lib/x86_64-linux-gnu/libpthread.so.0 ]; then
    echo "Using LD_PRELOAD workaround (alternative path)..."
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
else
    echo "Warning: Could not find system pthread library"
    echo "Trying without LD_PRELOAD..."
    rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
fi
