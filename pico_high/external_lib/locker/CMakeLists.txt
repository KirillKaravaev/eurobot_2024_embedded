add_library(locker INTERFACE)
target_sources(locker INTERFACE ${CMAKE_CURRENT_LIST_DIR}/locker.cpp)

target_include_directories(locker INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(locker INTERFACE pico_stdlib)
