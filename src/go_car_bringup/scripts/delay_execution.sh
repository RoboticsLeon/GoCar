#!/bin/bash          

# This bash script is used as a workaround in order to avoid unsync issues appearing in roslaunch,
# as Gazebo takes some time to start, to ensure algorithm are not called previous to
# complete loading of simulation.

# Arguments:
# 1-> Seconds to wait
# 2-> Package name where to find launch file
# 3-> Launch file name

echo "Preloading..."
sleep $1
echo "Preload completed!"
roslaunch $2 $3