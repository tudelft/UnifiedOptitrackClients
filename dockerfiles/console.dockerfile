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

# Base image
FROM ubuntu:22.04

# Update sources and install boost and build tools
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        libboost-all-dev \
        build-essential \
        cmake \
    # Remove apt cache
    && rm -rf /var/lib/apt/lists/*

# Add copy of local workspace 
WORKDIR /home/
ADD ./agents ./agents
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN mkdir build && cd build && cmake -D'AGENTS=console' .. && cmake --build .

# Add the entrypoint script
ADD ./dockerfiles/console_entrypoint.sh .
RUN chmod +x ./console_entrypoint.sh
ENTRYPOINT ["./console_entrypoint.sh"]
