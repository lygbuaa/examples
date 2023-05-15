cmake_minimum_required(VERSION 3.5)
project(cv_bridge_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(cv_bridge_demo_bin main.cpp)
ament_target_dependencies(cv_bridge_demo_bin 
  rclcpp 
  std_msgs
  cv_bridge
  image_transport
  geometry_msgs
  visualization_msgs
)

# install binary files
install(TARGETS
  cv_bridge_demo_bin
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
