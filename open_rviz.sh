#!/bin/bash
# Open RViz2 with EKF demo configuration
# Fix snap library conflict

cd ~/ekf_ws
source install/setup.bash

# Remove snap directories from LD_LIBRARY_PATH to avoid conflicts
LD_LIBRARY_PATH_FILTERED=""
IFS=':'
for path in $LD_LIBRARY_PATH; do
    case "$path" in
        *snap*) ;;
        *) LD_LIBRARY_PATH_FILTERED="${LD_LIBRARY_PATH_FILTERED}:${path}" ;;
    esac
done
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH_FILTERED#:}"

# Also try unsetting problematic environment variables
unset GTK_PATH
unset LIBGL_DRIVERS_PATH

exec rviz2 -d install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
