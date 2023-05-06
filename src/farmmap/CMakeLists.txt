cmake_minimum_required(VERSION 3.0.2)
project(farmmap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  darknet_ros_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

# find_package(PCL REQUIRED)

catkin_package(

)



include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/farmmap_node.cpp)
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
  #  ${PCL_LIBRARIES}
 )

