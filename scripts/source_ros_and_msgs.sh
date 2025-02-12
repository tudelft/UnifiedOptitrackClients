#!/bin/bash
# Copyright 2024 Anton Bredenbeck (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

# Check if ROS 2 is installed by checking for the existence of /opt/ros
if [ -d /opt/ros ]; then
    # Find the first directory under /opt/ros
    ros_distro=$(ls /opt/ros | head -n 1)

    # Source the setup.bash of the detected ROS 2 distro
    if [ -n "$ros_distro" ]; then
        source /opt/ros/$ros_distro/setup.bash
        echo "Sourced ROS 2 $ros_distro setup."
    else
        echo "No ROS 2 distribution found under /opt/ros."
        exit 1
    fi
else
    echo "ROS 2 is not installed on this machine."
    exit 1
fi

export LD_LIBRARY_PATH=../build/_deps/px4_msgs-build:${LD_LIBRARY_PATH}