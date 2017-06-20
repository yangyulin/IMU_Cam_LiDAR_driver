cmake_minimum_required(VERSION 2.8.3)
project(loitor_stereo_visensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  image_transport
  message_runtime
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)

include_directories(include 
${catkin_INCLUDE_DIRS}  
${OpenCV_INCLUDE_DIRS})

include_directories("./include/loitor_stereo_visensor")

add_library(loitorusbcam src/loitorusbcam.cpp)
target_link_libraries(loitorusbcam loitorimu)

add_library(loitorimu src/loitorimu.cpp)

add_executable(loitor_stereo_visensor src/loitor_stereo_visensor.cpp) 
target_link_libraries(loitor_stereo_visensor ${catkin_LIBRARIES})
target_link_libraries(loitor_stereo_visensor loitorusbcam loitorimu usb-1.0 ${OpenCV_LIBS})
add_dependencies(loitor_stereo_visensor loitor_vis_generate_messages_cpp) 


















