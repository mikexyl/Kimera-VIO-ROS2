cmake_minimum_required(VERSION 3.5)
project(kimera_vio_ros)

# set C++ standard to 20
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# turn off -Werror=unused-variable
add_compile_options(-Wno-unused-variable)

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
find_package(glog REQUIRED)
find_package(kimera_common REQUIRED)
find_package(kimera_vio REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(aria_viz REQUIRED)
find_package(aria_common REQUIRED)
find_package(jsoncpp REQUIRED)

kimera_package()

include_directories(
  include
)

set(dependencies
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(stereo_vio_node
  src/stereo_vio_node.cpp
  src/utils/geometry.cpp)
ament_target_dependencies(stereo_vio_node
  ${dependencies})
target_link_libraries(stereo_vio_node 
  kimera_vio
  aria_viz
  aria_common
)

set(executable_names
  stereo_vio_node
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
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skips the linter checks for copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(dependencies)
ament_export_include_directories(include)

# Print out fmt's include and library paths for debugging
message(STATUS "fmt_INCLUDE_DIRS: ${fmt_INCLUDE_DIRS}")
message(STATUS "fmt_LIBRARIES: ${fmt_LIBRARIES}")

ament_package()
