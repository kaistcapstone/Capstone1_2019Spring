cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project(balltracking)
add_executable(${PROJECT_NAME} "main.cpp")


find_package(OpenCV REQUIRED)
target_link_libraries(balltracking ${OpenCV_LIBS})
