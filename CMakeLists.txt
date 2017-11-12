cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_BUILD_TYPE "Debug")
project(fpfhtest)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (fpfhtest fpfhtest.cpp)
target_link_libraries (fpfhtest ${PCL_LIBRARIES})
 
add_executable (fpfh_filter_correspondences fpfh_filter_correspondences.cpp)
target_link_libraries (fpfh_filter_correspondences ${PCL_LIBRARIES})
