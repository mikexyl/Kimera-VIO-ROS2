cmake_minimum_required(VERSION 3.16)
project(mesh_splat)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(aria_viz REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(pcl_msgs REQUIRED)
find_package(projective_mesher_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(projective_mesher_node
  src/projective_mesher_node.cpp
)
target_include_directories(projective_mesher_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(projective_mesher_node
  aria_viz
  pcl_msgs
  projective_mesher_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(projective_mesher_node
  aria_viz
  gtsam
  ${OpenCV_LIBRARIES}
)

install(PROGRAMS
  scripts/dataset_recorder
  scripts/mesh_splat_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS projective_mesher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
