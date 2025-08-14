cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(LSM9DS1 C CXX ASM)



add_library(LSM9DS1 INTERFACE)

target_sources(LSM9DS1 INTERFACE 
                ${CMAKE_CURRENT_LIST_DIR}/LSM9DS1.cpp)

target_include_directories(LSM9DS1 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(LSM9DS1 INTERFACE
        pico_stdlib)

