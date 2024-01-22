# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Restate the arg to make it available in later stage
ARG ROS_DISTRO

# Update sources
RUN apt-get update && apt-get upgrade -y

# Install boost
RUN apt-get install libboost-all-dev -y

# Add copy of local workspace 
WORKDIR /home/
ADD ./clients ./clients
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && mkdir build && cd build && cmake -D'CLIENTS=ros2px4' .. && make

# Add the entrypoint script
ADD ./dockerfiles/ros2px4_entrypoint.sh .
RUN chmod +x ./ros2px4_entrypoint.sh
ENTRYPOINT ["./ros2px4_entrypoint.sh"]
