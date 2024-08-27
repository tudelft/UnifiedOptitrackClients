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


# Check if --filename or -o argument contains "/"
found_argument=
for arg in "$@"; do
    if [[ -n "$found_argument" || "$arg" == "-o"* || "$arg" == "--filename"* ]]; then
        if [[ $arg == *"/"* ]]; then
            echo "Error: when running with docker, --filename/-o has to be just a filename, no path allowed"
            exit 1
        fi
    fi

    if [[ "$arg" == "-o" || "$arg" == "--filename" ]]; then
        found_argument=t
    fi
done

cd /data
/home/build/mocap2log "$@"
