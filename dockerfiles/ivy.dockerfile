# Base image
FROM ubuntu:22.04

# Update sources
RUN apt-get update && apt-get upgrade -y

# Install boost and ivy
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y software-properties-common 
RUN apt-get install -y build-essential && apt-get install -y cmake
ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN add-apt-repository -y ppa:paparazzi-uav/ppa && apt-get update && apt-get install -y ivy-c-dev 

# Add copy of local workspace 
WORKDIR /home/
ADD ./clients ./clients
ADD ./include ./include
ADD ./src ./src
ADD ./scripts ./scripts
ADD ./CMakeLists.txt ./CMakeLists.txt

RUN mkdir build && cd build && cmake -D'CLIENTS=ivy' .. && make

# Add the entrypoint script
ADD ./dockerfiles/ivy_entrypoint.sh .
RUN chmod +x ./ivy_entrypoint.sh
ENTRYPOINT ["./ivy_entrypoint.sh"]
