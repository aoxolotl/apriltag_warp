cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("Save Point Cloud")

#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories ("/usr/include/eigen3")

link_directories("${PROJECT_SOURCE_DIR}/lib")

#add sources
file(GLOB SOURCES "./src/*.cpp")

add_executable(apriltag_det ${SOURCES})

target_link_libraries(apriltag_det libapriltags.a m pthread ${OpenCV_LIBS})
