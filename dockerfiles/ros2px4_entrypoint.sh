#!/bin/bash
. ./scripts/source_ros_and_msgs.sh
cd build && ./natnet2ros2px4 "$@"