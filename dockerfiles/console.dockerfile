# Base image
FROM ubuntu:22.04

# Update sources
RUN apt-get update && apt-get upgrade -y

# Install boost and build tools
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y build-essential && apt-get install -y cmake

# Add copy of local workspace 
WORKDIR /home/
ADD ./clients ./clients
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN mkdir build && cd build && cmake -D'CLIENTS=console' .. && make

# Add the entrypoint script
ADD ./dockerfiles/console_entrypoint.sh .
RUN chmod +x ./console_entrypoint.sh
ENTRYPOINT ["./console_entrypoint.sh"]
