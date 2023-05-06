cmake_minimum_required(VERSION 3.0.2)
project(tf_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  nav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

 add_executable(${PROJECT_NAME}_node src/tf_test_node.cpp)
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
