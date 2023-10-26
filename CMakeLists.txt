# Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity. Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_DEBUG)

cmake_minimum_required(VERSION 3.5)
project(optitrack_clients LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 11)


include(FetchContent)
fetchcontent_declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar
)
fetchcontent_makeavailable(natnet_sdk_content)

add_library(natnet_sdk SHARED IMPORTED)
set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)
target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)

include_directories(include)
add_executable(DebugImpl src/cyberzoo_mocap_client.cpp src/debug_impl.cpp)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(DebugImpl ${Boost_LIBRARIES})
target_link_libraries(DebugImpl natnet_sdk)