cmake_minimum_required(VERSION 3.8)
project(flexiv_grasp_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspTarget.msg"
  "msg/GraspTargetArray.msg"
  "srv/Detect.srv"
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(grasp_task_node src/grasp_task_node.cpp)
ament_target_dependencies(grasp_task_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  flexiv_msgs
  moveit_ros_planning_interface
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(grasp_task_node "${cpp_typesupport_target}")

install(TARGETS grasp_task_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/grasp_perception.py
  scripts/fake_gripper_server.py
  scripts/handeye_tf_publisher.py
  scripts/detect_probe.py
  scripts/save_frame.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
