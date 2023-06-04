cmake_minimum_required(VERSION 2.6)
project(MY_PROJECT)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.9.1 REQUIRED COMPONENTS common io geometry kdtree search features surface visualization filters)

file(GLOB SOURCES main.cpp JpegEncoder.cpp Frame.cpp)

add_executable(server ${SOURCES})
#include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories("/opt/libjpeg-turbo/include")
#link_directories("/opt/libjpeg-turbo/lib")


include_directories(server ${PCL_INCLUDE_DIRS} )
target_link_libraries(server ${PCL_LIBRARIES} turbojpeg zstd)


