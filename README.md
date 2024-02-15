Currently supported clients:

|        Client        | Means of Publishing                                                                  | Launch Command (Example)              |
|:--------------------:|------------------------------------------------------|---------------------------------------|
| `console` | Only to the terminal (if activated)                                                  | `natnet2console -c NED`                 |
| `ivy`                | For the Ivy client                                                                   | `natnet2ivy -s 123`                   |
| `udp`                | Data as a UDP stream                                                                 | `natnet2udp -i 192.168.209.100 -p 25` |
| `log`                | Data directly dumped into a file                                                     | `natnet2log -n myfile.csv`            |
| `ROS2`               | On two ros2 topics `/mocap/pose` and `/mocap/twist`                                  | `natnet2ros2 --publish_topic UAV`     |
| `ROS2PX4`            | As above + the published on the required PX4 topic `/fmu/in/vehicle_visual_odometry` | `natnet2ros2px4 -f 120`               |

Building natively
-------------------

Build all with:
```shell
mkdir build && cd build
cmake .. && make
```

Build only some with (for example):
```shell
mkdir build && cd build
cmake -D'CLIENTS=console;ivy;ros2;ros2px4' .. && make
```

## Prerequisites

Prerequisites vary per client. Currently, these are known:

|   Client  | Known Prerequisites                                                                      |
|:---------:|------------------------------------------------------------------------------------------|
| `ivy`     | `ivy-c-dev` installed from `ppa:paparazzi-uav/ppa`                                       |
| `ros2`    | `ros-humble-base`, needs to be sourced for compilation                                   |
| `ros2px4` | As above + `px4_msgs` must be sourced to run (execute `. scripts/source_ros_and_msgs.sh`)|

Build using Docker
------------------

Each client has its own `dockerfile` to make compilation across platforms easier. For this first install docker as is explained on the official [website](https://docs.docker.com/engine/install).

Then you can build your docker image using e.g., 

    docker build -t logclient . -f ./dockerfiles/log.dockerfile 

Where `-t` defines the name of the docker image and `-f` defines the file path to the `dockerfile`.
Afterward you can run the docker image with:

    docker run -it --rm --net host logclient -f cmdline args of your choice


How to write your own client?
==============================

To write your own client it has to inheret from the base class `UnifiedMocapClient` defined in `unified_mocap_client.hpp` and needs to implement the 

    void publish_data()
function. It _can_ also implement 

    void add_extra_po(boost::program_options::options_description &desc)
    void parse_extra_po(const boost::program_options::variables_map &vm)
Afterward, the client needs to be added to the `CMakeList.txt` file and added to the main executable `main.cpp` using compile options. A simple example of how to do this is the `ConsoleClient` defined in `clients/console_client.hpp`.
    
