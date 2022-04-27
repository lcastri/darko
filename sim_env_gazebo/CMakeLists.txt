cmake_minimum_required(VERSION 2.8.3)
project(sim_env_gazebo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)
find_package(ignition-math6 REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
    roscpp
    rospy
    gazebo_ros
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/ActorPlugin.cc
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME} ignition-math6::ignition-math6)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)