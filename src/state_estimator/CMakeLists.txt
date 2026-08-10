cmake_minimum_required(VERSION 3.8)
project(state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fm_driver REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- 自定义消息 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetState.msg"
  "msg/SystemState.msg"
  DEPENDENCIES std_msgs
)

# ---- 节点可执行文件 ----
add_executable(state_estimator_node src/state_estimator_node.cpp)

ament_target_dependencies(state_estimator_node
  rclcpp
  std_msgs
  nav_msgs
  fm_driver
  message_filters
  tf2
  tf2_geometry_msgs
)

target_link_libraries(state_estimator_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 让节点能 include 生成的消息头文件
target_include_directories(state_estimator_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

install(TARGETS
  state_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(state_estimator_node ${cpp_typesupport_target})

ament_package()
