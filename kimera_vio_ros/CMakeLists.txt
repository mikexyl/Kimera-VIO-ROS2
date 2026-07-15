cmake_minimum_required(VERSION 3.5)
project(kimera_vio_ros)

# turn off -Werror=unused-variable
add_compile_options(-Wno-error=unused-variable -Wno-error=unused-parameter -Wno-error=unused-but-set-variable)

# set c++20
set(CMAKE_CXX_STANDARD 20)

find_package(Boost REQUIRED COMPONENTS
  chrono
  date_time
  filesystem
  regex
  serialization
  system
  thread
  timer
)
find_package(Gflags REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(glog REQUIRED)
find_package(kimera_common REQUIRED)
find_package(kimera_vio REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(aria_viz REQUIRED)
find_package(aria_common REQUIRED)
find_package(xfeat-cpp REQUIRED)
find_package(pose_graph_tools_msgs REQUIRED)
find_library(SYSTEM_CURL_LIBRARY
  NAMES curl
  PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
  REQUIRED)

kimera_package()

include_directories(
  include
)

set(dependencies
  cv_bridge
  geometry_msgs
  kimera_vio
  message_filters
  nav_msgs
  pcl_conversions
  pcl_msgs
  rclcpp
  rclcpp_components
  pose_graph_tools_msgs
  sensor_msgs
  tf2_ros
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/components/stereo_vio.cpp
  src/components/mono_vio.cpp
  src/interfaces/backend_interface.cpp
  src/interfaces/base_interface.cpp
  src/interfaces/image_interface.cpp
  src/interfaces/imu_interface.cpp
  src/interfaces/local_loop_closure_publisher.cpp
  src/interfaces/multi_robot_loop_closure_bridge.cpp
  src/interfaces/stereo_interface.cpp
  src/interfaces/stereo_vio_interface.cpp
  src/interfaces/mono_interface.cpp
  src/interfaces/mono_vio_interface.cpp
  src/interfaces/RerunVisualizer.cpp
  src/utils/geometry.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies})
rclcpp_components_register_nodes(${library_name}
  "kimera_vio_ros::components::StereoVio"
  "kimera_vio_ros::components::MonoVio")
target_link_libraries(${library_name}
  kimera_vio aria_common aria_viz xfeat-cpp::xfeat-cpp ${SYSTEM_CURL_LIBRARY})

add_executable(stereo_vio_node
  src/stereo_vio_node.cpp)
ament_target_dependencies(stereo_vio_node
  ${dependencies})
target_link_libraries(stereo_vio_node
  ${library_name} ${SYSTEM_CURL_LIBRARY})

add_executable(mono_vio_node
  src/mono_vio_node.cpp)
ament_target_dependencies(mono_vio_node
  ${dependencies})
target_link_libraries(mono_vio_node
  ${library_name} ${SYSTEM_CURL_LIBRARY})

set(executable_names
  stereo_vio_node
  mono_vio_node
)

install(TARGETS ${executable_names} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # skips the linter checks for copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # The inherited launch and visualization sources predate the ament Python
  # and C++ format profiles. Retain syntax, XML, CMake and static checks.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_multi_robot_loop_closure_bridge
    test/test_multi_robot_loop_closure_bridge.cpp)
  if(TARGET test_multi_robot_loop_closure_bridge)
    target_link_libraries(test_multi_robot_loop_closure_bridge
      ${library_name} kimera_vio)
    ament_target_dependencies(test_multi_robot_loop_closure_bridge
      ${dependencies})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
    set_tests_properties(test_multi_robot_loop_closure_bridge PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs;ROS_LOCALHOST_ONLY=1")
  endif()
endif()

ament_export_dependencies(dependencies)
ament_export_include_directories(include)

ament_package()
