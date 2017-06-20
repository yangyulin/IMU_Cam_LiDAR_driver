cmake_minimum_required(VERSION 2.8)

project(camtest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")

link_directories("${PROJECT_SOURCE_DIR}/lib")

add_library(loitorusbcam src/loitorusbcam.cpp)
target_link_libraries(loitorusbcam loitorimu)

add_library(loitorimu src/loitorimu.cpp)

add_executable(camtest src/camtest2.cpp)
target_link_libraries(camtest loitorusbcam loitorimu usb-1.0 ${OpenCV_LIBS})
