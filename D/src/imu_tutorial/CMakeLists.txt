cmake_minimum_required(VERSION 2.8.3)
project(imu_tutorial)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)
add_executable(imu_tutorial_node src/imu_tutorial.cpp)
#add_dependencies(imu_tutorial_node imu_tutorial_generate_messages_cpp)

target_link_libraries(imu_tutorial_node
  ${catkin_LIBRARIES}
)

install(TARGETS imu_tutorial_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
