# Unified Mocap Client

This commandline program establishes connection to a motion capture system, 
processes the rigid-body pose information and publishes them for use with
different robotics systems.
```
┌─────────┐             ┌────────────────────────┐                     
│ Motion  │────────────►│                        │  various    ┌──────────┐
│ Capture │  TCP / UDP  │  Unified Mocap Client  ┼───────────► | Agent(s) |
│ System  │◄────────────┤                        │ protocols   └──────────┘ 
└─────────┘             └────────────────────────┘                     
```

## Support

### Currently supported motion capture systems:
* NaturalPoint, Inc. **"OptiTrack"** via its `NatNet SDK`


### Currently supported agents:

|        Agent        | Means of Publishing                                                                  | Launch Command (Example)               |
|:--------------------:|--------------------------------------------------------------------------------------|----------------------------------------|
| `ROS2`               | On two ros2 topics `/mocap/pose` and `/mocap/twist`                                  | `mocap2ros2 --publish_topic UAV`       |
| `ROS2PX4`            | As above + the published on the required PX4 topic `/fmu/in/vehicle_visual_odometry` | `mocap2ros2px4 -f 120`                 |
| `mavlink`            | Stream data as MAVLink package over udp for PX4, Arducopter, and (soon) Arduplane    | `mocap2mavlink -s 123 --autopilot px4` |
| `udp`                | Data as a UDP stream                                                                 | `mocap2udp -i 192.168.209.100 -p 1234`   |
| `ivy`                | Publish to an IVY bus                                                                   | `mocap2ivy -s 123`                     |
| `console`            | Only to the terminal (if activated)                                                  | `mocap2console -c NED`                 |
| `log`                | Data directly dumped into a file                                                     | `mocap2log -n myfile.csv`              |

### Currently supported platforms:

Ubuntu >= 22.04 on x86


## Building and Running

### Natively

Prerequisites vary per agent. Currently, these are known:

|   Agent   | Known Prerequisites                                                                      |
|:---------:|------------------------------------------------------------------------------------------|
| `all`     | `libboost-all-dev` installed with `apt`                                       |
| `ivy`     | `ivy-c-dev` installed from `ppa:paparazzi-uav/ppa`                                       |
| `mavlink` | `python3-pip` installed with apt                                                         |
| `ros2`    | `ros-humble-base`, needs to be sourced for compilation                                   |
| `ros2px4` | As above + `px4_msgs` must be sourced to run (execute `. scripts/source_ros_and_msgs.sh`)|

Build for all agents with:
```shell
mkdir build && cd build
cmake .. && make
```

Build only for some agents with (for example):
```shell
mkdir build && cd build
cmake -D'AGENTS=console;ivy;ros2;ros2px4' .. && make
```

Run with (see table at beginning on this readme)
```shell
./mocap2console --help
```

### Alternative: Build and run using Docker

Each agent has its own `dockerfile` to make compilation across platforms easier. For this first install docker as is explained on the official [website](https://docs.docker.com/engine/install).

Then you can build your docker image using e.g., 

    docker build -t consoleagent . -f ./dockerfiles/console.dockerfile 

Where `-t` defines the name of the docker image and `-f` defines the file path to the `dockerfile`.
Afterward, you can run the docker image with:

    docker run -it --rm --net host consoleagent cmdline args of your choice

When running the logging agent with docker you have to mount a volume such that the written file will persist on the host machine. 
The agent expects the volume to be mounted under `/data`, i.e. the command would be something like to save the data in the current directory:

    docker run -it --net=host -v ./:/data logagent -s 1 -o data.csv


### Notes on using this for MAVLink agents

Possibly the best way to forward the MAVLink UDP stream to your drone is to use [mavlink-router](https://github.com/mavlink-router/mavlink-router). You can download the latest glibc-x86 version from the release page of the github repo.

The router connects to the drone via a serial telemetry radio device, or a UDP address (e.g. for Herelink). You then instruct the router to connect two other local UDP endpoints; one for unified_mocap_client and one for your groundstation (e.g. QGroundControl or MissionPlanner):

    ./mavlink-routerd-glibc-x86_64 -t 0 -e 127.0.0.1:14553 -e 127.0.0.1:14554 /dev/ttyUSB0:57600

In a different terminal, start the mocap client:

    ./mocap2mavlink -s 1 -c NED -f 10 -p 14554 --autopilot arducopter

You can now open your groundstation and connect to UDP port 14553.

__Note__: it is currently untested what happens if multiple UAV are on the same
mavlink network routed by mavlink-routerd!


## History

* 2024-08-27 -- 1.0.0 -- First Release

## Authors

* Anton Bredenbeck ([@antbre](https://github.com/antbre), Delft University of Technology, a.bredenbeck - @ - tudelft.nl)
* Till Blaha ([@tblaha](https://github.com/tblaha), Delft University of Technology, t.m.blaha - @ - tudelft.nl)

## Structure
```
.
├── agents          # contains the agent definitions
├── dockerfiles      # contains dockerfiles to build for each agent
├── include          # headers
├── scripts          # utility scripts
├── src              # implementation of data receiving and pose calculations
├── CMakeLists.txt
└── README.md
```

## License

[![License](https://img.shields.io/badge/License-GPL--3.0--or--later-4398cc.svg?logo=spdx)](https://spdx.org/licenses/GPL-3.0-or-later.html)

The contents of this repository are licensed under **GNU General Public License v3.0** (see `LICENSE` file).

Technische Universiteit Delft hereby disclaims all copyright interest in the
program “Unified Mocap Client" (one line description of the content or function)
written by the Author(s).

Henri Werij, Dean of the Faculty of Aerospace Engineering

© 2024, Anton Bredenbeck, Till Blaha


## How to add support for a different agent?

To add support for your own agent it has to inheret from the base class `UnifiedMocapClient` defined in `unified_mocap_client.hpp` and needs to implement the 

    void publish_data()
function. It _can_ also implement 

    void add_extra_po(boost::program_options::options_description &desc)
    void parse_extra_po(const boost::program_options::variables_map &vm)
Afterward, the agent needs to be added to the `CMakeList.txt` file and added to the main executable `main.cpp` using compile options. A simple example of how to do this is the `ConsoleAgent` defined in `agents/console_agent.hpp`.

## How to add support for a different Motion Capture System?

Please open an Issue and we'll support you!


# FAQ

ROS: source workspace and import libraries

```
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=../build/_deps/px4_msgs-build:${LD_LIBRARY_PATH}
```