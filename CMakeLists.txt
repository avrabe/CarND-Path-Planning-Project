project(Path_Planning)

cmake_minimum_required (VERSION 3.2)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")

    set(CXX_FLAGS "-Wall  -O0 -fprofile-arcs -ftest-coverage ")
    set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

endif(${CMAKE_BUILD_TYPE} MATCHES "Debug")


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS pthread dl)
