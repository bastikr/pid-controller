cmake_minimum_required(VERSION 2.6)
project(pidcontroller)

add_executable(pidcontroller main.cpp)

install(TARGETS pidcontroller RUNTIME DESTINATION bin)
