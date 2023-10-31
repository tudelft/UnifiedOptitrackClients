# Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity. Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_DEBUG)

cmake_minimum_required(VERSION 3.14)

# project and executables
project(optitrack_clients LANGUAGES CXX)
# Define a cache variable as a string
if(NOT DEFINED CLIENTS)
    message(STATUS "CLIENTS is not set. Continuing with all clients")
    set(CLIENTS "debug;ivy" CACHE STRING "Clients to build into the binary")
endif(NOT DEFINED CLIENTS)

add_executable(client src/main.cpp src/cyberzoo_mocap_client.cpp)

foreach(x ${CLIENTS})
    string(TOUPPER ${x} uppercase)
    message(STATUS ${uppercase})
    message(STATUS ${x})
    add_definitions(-DUSE_CLIENT_${uppercase})
    ADD_CUSTOM_TARGET(${x}_symlink ALL
                      COMMAND ${CMAKE_COMMAND} -E create_symlink client natnet2${x}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endforeach()

include_directories(include)
include_directories(clients)

# compiler
set (CMAKE_CXX_STANDARD 14)
if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find external libraries and include for all clients
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(client ${Boost_LIBRARIES})

include(FetchContent)
fetchcontent_declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar
)
fetchcontent_makeavailable(natnet_sdk_content)

add_library(natnet_sdk SHARED IMPORTED)
set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)
target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)
target_link_libraries(client natnet_sdk)


# client specific stuff
if ("ivy" IN_LIST CLIENTS)
    add_definitions(-DUSE_DISCOVERY)
    find_library(IVY_LIB NAMES ivy)
    target_link_libraries(client ivy)

    find_package(PkgConfig REQUIRED)
    pkg_search_module(GLIB REQUIRED glib-2.0)

    include_directories(${GLIB_INCLUDE_DIRS})
    link_directories(${GLIB_LIBRARY_DIRS})

    add_definitions(${GLIB_CFLAGS_OTHER})
    target_link_libraries(client ${GLIB_LIBRARIES})
endif()
