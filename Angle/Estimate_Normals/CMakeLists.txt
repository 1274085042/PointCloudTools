cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extimate_normals)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable (estimate_normals estimate_normals.cpp)
target_link_libraries (estimate_normals ${PCL_LIBRARIES})