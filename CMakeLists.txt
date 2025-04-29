cmake_minimum_required(VERSION 3.0.2)
project(gravity_plane_est)

# Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add the executable for the subscriber node
add_executable(avg_imu cpp/avg_imu.cpp)

# Link the executable to the necessary libraries
target_link_libraries(avg_imu
  ${catkin_LIBRARIES}
)
