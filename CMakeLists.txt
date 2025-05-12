cmake_minimum_required(VERSION 3.10)

project(roboteqlibs)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(BOOST_ROOT "C:/Users/CH253140/Downloads/boost_1_86_0/boost_1_86_0")
set(Boost_LIBRARIES "C:/Users/CH253140/Downloads/boost_1_86_0/boost_1_86_0/stage/lib")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)

set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
list(APPEND serial_SRCS src/impl/win.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)

add_library(roboteqlibs
    src/RoboteqDriver.cpp
    src/serial.cc
    ${serial_SRCS}
)

target_link_libraries(roboteqlibs ${Boost_LIBRARIES}) # gtest rt pthread

target_include_directories(roboteqlibs
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
    PRIVATE
        ${Boost_INCLUDE_DIRS}
)

add_executable(sample 
    examples/serialtest.cpp
)

target_include_directories(sample PUBLIC
                           "${PROJECT_BINARY_DIR}"
)


target_link_libraries(sample
    PRIVATE
        roboteqlibs
)
