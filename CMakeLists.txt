cmake_minimum_required(VERSION 3.0.2)
project(flappy_bird_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  roscpp
  roslint
)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(flappyBirdPlugin SHARED src/flappyBirdPlugin.cpp)
target_link_libraries(flappyBirdPlugin ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES} yaml-cpp
)

add_library(CameraMoverPlugin SHARED src/CameraMoverPlugin.cpp)
target_link_libraries(CameraMoverPlugin ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/flyer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(ROSLINT_PYTHON_OPTS "1>&2")

roslint_cpp()
roslint_python()
