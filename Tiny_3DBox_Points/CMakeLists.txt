cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(TinyBoxStatistic)


find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(tinyboxstatistic tinyboxstatistic.cpp)
target_link_libraries(tinyboxstatistic ${PCL_LIBRARIES})