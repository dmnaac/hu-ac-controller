cmake_minimum_required(VERSION 3.8)
project(uwb_wma_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fm_driver REQUIRED)

add_executable(uwb_wma_filter_node src/uwb_wma_filter_node.cpp)

target_include_directories(uwb_wma_filter_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(uwb_wma_filter_node
  rclcpp
  std_msgs
  fm_driver
)

install(TARGETS
  uwb_wma_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
