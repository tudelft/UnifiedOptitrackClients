# Base image
FROM ubuntu:22.04

# Update sources and install boost and build tools
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        libboost-all-dev \
        build-essential \
        cmake \
        git \
        python3-pip \
    # Remove apt cache
    && rm -rf /var/lib/apt/lists/*

# Add copy of local workspace 
WORKDIR /home/
ADD ./clients ./clients
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN mkdir build && cd build && cmake -D'CLIENTS=mavlink' .. && cmake --build .

# Add the entrypoint script
ADD ./dockerfiles/mavlink_entrypoint.sh .
RUN chmod +x ./mavlink_entrypoint.sh
ENTRYPOINT ["./mavlink_entrypoint.sh"]