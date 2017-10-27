cmake_minimum_required(VERSION 2.6)
project(pidcontroller)

add_executable(example main.cpp)

include_directories(include)
add_library(pidcontroller src/pidcontroller.cpp)
target_link_libraries(example pidcontroller)
# install(TARGETS pidcontroller RUNTIME DESTINATION bin)
