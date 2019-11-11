cmake_minimum_required(VERSION 2.8.3)
project(rgbd_tutorial)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  image_transport
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  #include
)
add_executable(rgbd_tutorial_node src/rgbd_tutorial.cpp)
#add_dependencies(imu_tutorial_node imu_tutorial_generate_messages_cpp)

target_link_libraries(rgbd_tutorial_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

#install(TARGETS lidar_tutorial_node
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
