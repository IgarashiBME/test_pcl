cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PROJECT)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_down pcd_down)
target_link_libraries(pcd_down ${PCL_LIBRARIES})
