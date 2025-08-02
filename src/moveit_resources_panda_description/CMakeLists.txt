cmake_minimum_required(VERSION 3.5)
project(moveit_resources_panda_description)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Declare executable: fk_node
# add_executable(fk_node src/fk_node.cpp)
# add_executable(ik_node src/ik_node.cpp)

# Link dependencies
# ament_target_dependencies(fk_node rclcpp trajectory_msgs)
# ament_target_dependencies(ik_node rclcpp trajectory_msgs)


# Include Eigen headers
# target_include_directories(fk_node PRIVATE ${EIGEN3_INCLUDE_DIR})
# target_include_directories(ik_node PRIVATE ${EIGEN3_INCLUDE_DIR})

# Install executable
# install(TARGETS
#   fk_node
#   ik_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install other directories (URDF, meshes, config, launch)
install(
  DIRECTORY 
  environment
  cmake
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
