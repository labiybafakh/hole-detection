cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(HoleDetectionViewer)

# Set C++14 to ensure compatibility with PCL and RealSense
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find PCL package
find_package(PCL 1.8 REQUIRED)

if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL not found.")
endif()

# Include PCL libraries
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find RealSense package
find_package(realsense2 REQUIRED)

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "RealSense2 not found.")
endif()

# Add the executable
add_executable(hole_detection_viewer src/viewer.cpp)

# Link the executable to the required PCL and RealSense libraries
target_link_libraries(hole_detection_viewer ${PCL_LIBRARIES} ${realsense2_LIBRARY})
