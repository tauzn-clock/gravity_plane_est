cmake_minimum_required(VERSION 3.0.2)
project(gravity_plane_est)

# Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find the YAML-CPP library (for YAML support)
find_package(yaml-cpp REQUIRED)

# Add the executable for the subscriber node
add_executable(avg_imu cpp/avg_imu.cpp)
add_executable(find_planes cpp/find_planes.cpp)

# Link the executable to the necessary libraries
target_link_libraries(avg_imu
  ${catkin_LIBRARIES}
)

target_link_libraries(find_planes
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

# Link python executable
catkin_install_python(PROGRAMS
  python/find_planes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
