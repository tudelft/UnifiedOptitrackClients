# Copyright 2024 Anton Bredenbeck, Till Blaha (Delft University of Technology)
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

cmake_minimum_required(VERSION 3.14)

# project and executables
project(optitrack_clients LANGUAGES CXX)

file(GLOB SOURCES "src/*.cpp")
add_executable(client ${SOURCES})

# ADD YOUR NEW AGENT TO THIS SEMICOLON-SEPARATED LIST!
if(NOT DEFINED MOCAPS)
    message(STATUS "MOCAPS is not set. Continuing with all mocaps")
    #set(MOCAPS "test,optitrack,vicon,qualisys" CACHE STRING "Mocaps to build into the binary")
    set(MOCAPS "test" CACHE STRING "Mocaps to build into the binary")
endif(NOT DEFINED MOCAPS)

if(NOT DEFINED AGENTS)
    message(STATUS "AGENTS is not set. Continuing with all agents")
    #set(AGENTS "console;ivy;udp;log;ros2;ros2px4" CACHE STRING "Agents to build into the binary")
    set(AGENTS "console;udp" CACHE STRING "Agents to build into the binary")
endif(NOT DEFINED AGENTS)

foreach(x ${AGENTS})
    string(TOUPPER ${x} uppercase)
    add_definitions(-DUSE_AGENT_${uppercase})
    #ADD_CUSTOM_TARGET(${x}_symlink ALL
    #                  COMMAND ${CMAKE_COMMAND} -E create_symlink client mocap2${x}
    #                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endforeach()

foreach(x ${MOCAPS})
    string(TOUPPER ${x} uppercase)
    add_definitions(-DUSE_MOCAP_${uppercase})
endforeach()

include_directories(include)
include_directories(agents)
include_directories(mocaps)
include_directories(scripts)

# compiler
set (CMAKE_CXX_STANDARD 14)
if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find external libraries and include for all agents
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(client ${Boost_LIBRARIES})

if ("optitrack" IN_LIST MOCAPS)
    include(FetchContent)
    fetchcontent_declare(natnet_sdk_content
      URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.1_ubuntu.tar
    )
    fetchcontent_makeavailable(natnet_sdk_content)

    add_library(natnet_sdk SHARED IMPORTED)
    set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)
    target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)
    target_link_libraries(client natnet_sdk)
endif()

if ("vicon" IN_LIST MOCAPS)
    include(FetchContent)
    fetchcontent_declare(vicon_sdk_content
      URL https://app.box.com/shared/static/7s2j1j2oec1d19up2p5se5tlqrsky149.zip
    )
    fetchcontent_makeavailable(vicon_sdk_content)

    add_library(vicon_sdk SHARED IMPORTED)
    set_target_properties(vicon_sdk PROPERTIES IMPORTED_LOCATION ${vicon_sdk_content_SOURCE_DIR}/20230413_145507h/Release/Linux64/libViconDataStreamSDK_CPP.so)
    target_include_directories(vicon_sdk INTERFACE ${vicon_sdk_content_SOURCE_DIR}/20230413_145507h/Release/Linux64/)
    target_link_libraries(client vicon_sdk)
endif()

if ("qualisys" IN_LIST MOCAPS)
    find_package(Boost REQUIRED COMPONENTS system)
endif()

# Client-specific stuff
if ("ivy" IN_LIST AGENTS)
    find_library(IVY_LIB NAMES ivy)
    target_link_libraries(client ivy)

    find_package(PkgConfig REQUIRED)
    pkg_search_module(GLIB REQUIRED glib-2.0)

    include_directories(${GLIB_INCLUDE_DIRS})
    link_directories(${GLIB_LIBRARY_DIRS})

    add_definitions(${GLIB_CFLAGS_OTHER})
    target_link_libraries(client ${GLIB_LIBRARIES})
endif()

if ("udp" IN_LIST AGENTS)
    find_package(Boost REQUIRED COMPONENTS system)
endif()

if ("ros2" IN_LIST AGENTS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)

    ament_target_dependencies(client rclcpp geometry_msgs)
endif()

if ("ros2px4" IN_LIST AGENTS)
    # px4msgs is BSD 3 clause licensed
    FetchContent_Declare(
      px4_msgs
      GIT_REPOSITORY https://github.com/PX4/px4_msgs
      GIT_TAG release/1.14
    )
    FetchContent_MakeAvailable(px4_msgs)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)

    add_dependencies(client px4_msgs)
    ament_target_dependencies(client rclcpp geometry_msgs)
    target_link_libraries(client ${CMAKE_BINARY_DIR}/_deps/px4_msgs-build/libpx4_msgs__rosidl_typesupport_cpp.so)
    target_include_directories(client PUBLIC ./build/_deps/px4_msgs-build/rosidl_generator_cpp)
endif()
