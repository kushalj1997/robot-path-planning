cmake_minimum_required(VERSION 3.8)
project(RobotPathPlan)

# Compile in C++ 17 (latest standard)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The version number
set(RobotPathPlan_VERSION_MAJOR 1)
set(RobotPathPlan_VERSION_MINOR 0)

# Find OpenCV on the host machine
find_package(OpenCV REQUIRED)
# add the executable for mapper and planner in one class
add_executable(RobotPathPlan Obstacle.cpp RobotMapPlan.cpp main.cpp)

# link OpenCV and Boost libraries to the project
target_link_libraries(RobotPathPlan ${OpenCV_LIBS})
