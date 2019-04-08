set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

project(threadmill)

include_directories(include
                    /usr/include/eigen3)

file(GLOB SOURCES "src/*.c*")

#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

set (SRC_CPP ${SOURCES})
add_library(funccpp_lib STATIC ${SRC_CPP})

add_executable(test_cpp src/main.cpp)
target_link_libraries(test_cpp ${PROJECT_LINK_LIBS} funccpp_lib)

add_executable(test_c src/main.c)
target_link_libraries(test_c ${PROJECT_LINK_LIBS} funccpp_lib)

add_executable(control_test_cpp src/control_test.cpp)
target_link_libraries(control_test_cpp ${PROJECT_LINK_LIBS} funccpp_lib)

add_executable(control_test_c src/control_test.c)
target_link_libraries(control_test_c ${PROJECT_LINK_LIBS} funccpp_lib)
