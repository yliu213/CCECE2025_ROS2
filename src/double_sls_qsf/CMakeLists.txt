cmake_minimum_required(VERSION 3.8)
project(double_sls_qsf)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(setup REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

#################
# Setup targets #
#################

# add executables
add_executable(sls_qsf src/main_node.cpp)
ament_target_dependencies(sls_qsf rclcpp px4_msgs)
# find executables
install(TARGETS sls_qsf DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############
# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp px4_msgs)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()