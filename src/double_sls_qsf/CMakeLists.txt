cmake_minimum_required(VERSION 3.8)
project(double_sls_qsf)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(setup REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

#################
# Setup targets #
#################

# add executables
add_executable(sls_qsf 
  src/main_node.cpp
  # for MATLAB generated code
  src/QSFGeometricController/QSFGeometricController.cpp
  src/QSFGeometricController/rt_nonfinite.cpp
  src/QSFGeometricController/rtGetInf.cpp
  src/QSFGeometricController/rtGetNaN.cpp
  # for inner loop controller
  src/nonlinear_attitude_control.cpp
)
ament_target_dependencies(sls_qsf 
  rclcpp 
  px4_msgs 
  gazebo_msgs 
  geometry_msgs 
  controller_msgs
  px4_ros_com
)

# Make sure Eigen's include path is used
target_include_directories(sls_qsf PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${px4_ros_com_INCLUDE_DIRS}  # Add PX4 frame transforms
)

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