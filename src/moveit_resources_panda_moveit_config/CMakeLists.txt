cmake_minimum_required(VERSION 3.22)
project(moveit_resources_panda_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(panda_ik_node scripts/panda_ik_node.cpp)
ament_target_dependencies(panda_ik_node rclcpp trajectory_msgs geometry_msgs)
target_link_libraries(panda_ik_node Eigen3::Eigen)

add_executable(panda_fk_node scripts/panda_fk_node.cpp)
ament_target_dependencies(panda_fk_node rclcpp trajectory_msgs)
target_link_libraries(panda_fk_node Eigen3::Eigen)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(TARGETS panda_ik_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS panda_fk_node
  DESTINATION lib/${PROJECT_NAME}
)
