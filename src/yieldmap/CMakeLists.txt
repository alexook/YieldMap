cmake_minimum_required(VERSION 3.0.2)
project(yieldmap)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  cv_bridge
  sensor_msgs
  image_geometry
  image_transport
  pcl_ros

  darknet_ros_msgs
)


find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
)


include_directories(
src
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
)


add_executable(yield src/yield_node.cpp
                      src/yield_map.cpp
                      src/CameraPoseVisualization.cpp
              )
target_link_libraries(yield darknet ${OpenCV_LIBRARIES} Threads::Threads ${catkin_LIBRARIES})
