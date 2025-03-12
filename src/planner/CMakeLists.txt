cmake_minimum_required(VERSION 3.0.2)
project(planner)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
)

# Catkin specific configuration
catkin_package()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Declare C++ library
add_library(gfm_planner
    src/planner/astar.cpp
    src/planner/minco.cpp
)

# Declare C++ executables
add_executable(costmap src/costmap.cpp)
add_executable(planner src/planner.cpp)

target_link_libraries(costmap
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

target_link_libraries(planner
    gfm_planner
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
