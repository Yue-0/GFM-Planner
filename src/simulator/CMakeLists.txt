cmake_minimum_required(VERSION 3.0.2)
project(simulator)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    geometry_msgs
    sensor_msgs
    nav_msgs
    roscpp
    tf
)

# Catkin specific configuration
catkin_package()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(map src/map.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(map
    ${catkin_LIBRARIES}
)

# Mark executable scripts for installation
catkin_install_python(PROGRAMS
    scripts/lidar.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
    scripts/metric.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
    scripts/architect.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
    scripts/initialpose.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
    scripts/localization.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
