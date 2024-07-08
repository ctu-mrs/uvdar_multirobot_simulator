cmake_minimum_required(VERSION 3.5)
project(uvdar_multirobot_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-fno-omit-frame-pointer)

set(CATKIN_DEPENDENCIES
  message_generation
  message_runtime
  roscpp
  mrs_msgs
  nav_msgs
  std_msgs
  mrs_lib
  )

set(EXECUTABLES
  uvdar_sim
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_messages(DEPENDENCIES
  mrs_msgs
  std_msgs
  sensor_msgs
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${LIBGBM_INCLUDE_DIRS}
  )

add_executable(uvdar_sim
  src/uvdar_sim.cpp
  )

add_dependencies(uvdar_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(uvdar_sim
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )
