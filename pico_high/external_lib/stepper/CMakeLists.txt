add_library(stepper INTERFACE)
target_sources(stepper INTERFACE ${CMAKE_CURRENT_LIST_DIR}/stepper.cpp)

target_include_directories(stepper INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(stepper INTERFACE pico_stdlib hardware_pwm)
