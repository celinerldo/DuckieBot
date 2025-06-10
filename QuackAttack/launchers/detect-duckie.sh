#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun followlane detect_duckie_node.py

# wait for app to end
dt-launchfile-join