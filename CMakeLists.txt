cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("Save Point Cloud")

#set(CMAKE_CXX_FLAGS "-std=c++11")
# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")

link_directories("${PROJECT_SOURCE_DIR}/lib")

#add sources
file(GLOB SOURCES "./src/*.cpp")

add_executable(apriltag_det ${SOURCES})

target_link_libraries(apriltag_det libapriltag.a m pthread ${OpenCV_LIBS})
