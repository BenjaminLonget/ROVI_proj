cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(rovi2)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ex1 ex1.cpp)
add_executable(ex2 ex2.cpp)

target_link_libraries(ex1 ${PCL_LIBRARIES})
target_link_libraries(ex2 ${PCL_LIBRARIES})
