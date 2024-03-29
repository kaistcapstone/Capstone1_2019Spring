cmake_minimum_required(VERSION 2.8.3)
project(ball_detection)
add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport
  message_generation
  visualization_msgs
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  image_transport
#  visualization_msgs
#  DEPENDS system_lib
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #include
)


add_executable(ball_detect_node src/ball_detect1.cpp)
add_dependencies(ball_detect_node core_msgs_generate_messages_cpp)

target_link_libraries(ball_detect_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(ball_detect_near_node src/ball_detect.cpp)
add_dependencies(ball_detect_near_node core_msgs_generate_messages_cpp)

target_link_libraries(ball_detect_near_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)



add_executable(ball_detect_back2_node src/ball_detect_back2.cpp)
add_dependencies(ball_detect_back2_node core_msgs_generate_messages_cpp)


target_link_libraries(ball_detect_back2_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)


