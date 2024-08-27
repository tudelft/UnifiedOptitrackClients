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
