add_subdirectory(MPU6050)

add_library(imu INTERFACE)
target_sources(imu INTERFACE ${CMAKE_CURRENT_LIST_DIR}/imu.cpp)

target_include_directories(imu INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(imu INTERFACE pico_stdlib I2Cdev MPU6050)
