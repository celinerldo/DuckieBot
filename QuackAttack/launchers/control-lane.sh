#!/bin/bash

# Source the environment
source /environment.sh

# Initialize the launch file
dt-launchfile-init

# --- YOUR CODE BELOW THIS LINE ---

echo "Launching Driving Nodes in Background..."
# Launch the lane detection node in the background
rosrun followlane detect_lane_node.py &

# Launch the switch node in the background
rosrun followlane switch_control_node.py &

# Launch the lane control node in the background
rosrun followlane control_lane_node.py &

echo "Launching Camera GUI Node..."
# Launch the camera reader GUI node as the main process
# dt-exec will make this the "main" process for joining/shutdown
dt-exec rosrun followlane camera_reader_node.py

# --- YOUR CODE ABOVE THIS LINE ---

# Wait for the main app (dt-exec) to end
dt-launchfile-join