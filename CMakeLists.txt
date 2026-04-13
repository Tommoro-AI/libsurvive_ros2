# Copyright 2022 Andrew Symington
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Declare the minimum cmake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project(libsurvive_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set compile options
add_compile_options(-Wall -Wextra -Wpedantic)

# Fetch and build libsurvive during configure.
include(FetchContent)
FetchContent_Declare(
  libsurvive_src
  GIT_REPOSITORY https://github.com/cntools/libsurvive.git
  GIT_TAG 2f4303c76f53be1919d89d801172fddd7b09aa7a)
FetchContent_MakeAvailable(libsurvive_src)

if(TARGET survive)
  set(LIBSURVIVE_TARGET survive)
elseif(TARGET libsurvive)
  set(LIBSURVIVE_TARGET libsurvive)
else()
  message(
    FATAL_ERROR
      "Could not find a libsurvive CMake target after FetchContent_MakeAvailable()."
  )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Universally add this components includes
include_directories(include)

# Add the component
add_library(libsurvive_ros2_component SHARED src/component.cpp)
target_include_directories(
  libsurvive_ros2_component
  PUBLIC ${libsurvive_src_SOURCE_DIR}/include
         ${libsurvive_src_SOURCE_DIR}/include/libsurvive
         ${libsurvive_src_SOURCE_DIR}/redist)
target_link_libraries(libsurvive_ros2_component ${LIBSURVIVE_TARGET})
target_compile_definitions(libsurvive_ros2_component
                           PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(
  libsurvive_ros2_component
  rclcpp
  rclcpp_components
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros)
rclcpp_components_register_nodes(libsurvive_ros2_component
                                 "libsurvive_ros2::Component")

# Export the library path because the package installs libraries without
# exporting them
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# Manually compose the component into a node
add_executable(libsurvive_ros2_node src/node.cpp)
target_link_libraries(libsurvive_ros2_node libsurvive_ros2_component)
ament_target_dependencies(libsurvive_ros2_node rclcpp)

# Publish static world<->libsurvive_world alignment from joy button events
add_executable(libsurvive_world_align_node src/world_align_node.cpp)
ament_target_dependencies(
  libsurvive_world_align_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros)

# Install the component
install(
  TARGETS libsurvive_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install the node
install(TARGETS libsurvive_ros2_node DESTINATION lib/${PROJECT_NAME})

# Install the world alignment helper node
install(TARGETS libsurvive_world_align_node DESTINATION lib/${PROJECT_NAME})

# Install the shared things
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# For testing only
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export all include directories and declare the package
ament_export_include_directories(include)
ament_package()
