add_library(rpm INTERFACE)
target_sources(rpm INTERFACE ${CMAKE_CURRENT_LIST_DIR}/rpm.cpp)

target_include_directories(rpm INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(rpm INTERFACE pico_stdlib hardware_pwm)
