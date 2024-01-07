cmake_minimum_required(VERSION 3.0)
project(bno055)

add_library(i2c i2c.c)
add_library(bno055_i2c bno055_i2c_lib.cpp)
target_link_libraries(bno055_i2c i2c)

add_executable(bno055 bno055.cpp)
target_link_libraries(bno055 PRIVATE bno055_i2c)
target_include_directories(bno055 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
