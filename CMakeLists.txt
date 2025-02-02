cmake_minimum_required(VERSION 3.10)
project(bno055)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(bno055_i2c bno055_i2c_lib.cpp)
target_include_directories(bno055_i2c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(bno055_i2c PRIVATE pthread)

add_executable(bno055 bno055.cpp)
target_link_libraries(bno055 PRIVATE bno055_i2c)

target_include_directories(bno055 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(record_imu_data record_imu_data.cpp)
target_link_libraries(record_imu_data PRIVATE bno055_i2c)
target_include_directories(record_imu_data PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
