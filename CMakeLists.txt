# Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity. Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_DEBUG)

cmake_minimum_required(VERSION 3.10)
project(optitrack_clients LANGUAGES C CXX)
set (CMAKE_CXX_STANDARD 11)

find_library(IVY_LIB NAMES ivy)

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

include(FetchContent)
fetchcontent_declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar
)
fetchcontent_makeavailable(natnet_sdk_content)

add_library(natnet_sdk SHARED IMPORTED)
set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)
target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-DUSE_DISCOVERY)
include_directories(include)


add_executable(DebugImpl src/cyberzoo_mocap_client.cpp src/debug_impl.cpp)
target_link_libraries(DebugImpl ${Boost_LIBRARIES})
target_link_libraries(DebugImpl natnet_sdk)

add_definitions(-DDOWNLINK)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ext/pprzlink/var/include)
add_executable(NatNet2Ivy src/cyberzoo_mocap_client.cpp src/ivy.cpp)
target_link_libraries(NatNet2Ivy ${Boost_LIBRARIES})
target_link_libraries(NatNet2Ivy natnet_sdk)
target_link_libraries(NatNet2Ivy ivy)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)

include_directories(${GLIB_INCLUDE_DIRS})
link_directories(${GLIB_LIBRARY_DIRS})

add_definitions(${GLIB_CFLAGS_OTHER})
target_link_libraries(NatNet2Ivy ${GLIB_LIBRARIES})
