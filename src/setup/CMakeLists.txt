# cmake_minimum_required(VERSION 3.8)
# project(setup)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# add_executable(hello src/hello.cpp)
# target_include_directories(hello PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(hello PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS hello
#   DIRECTORY models worlds
#   DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

#####################################
cmake_minimum_required(VERSION 3.8)
project(setup)

# Remove compiler flags unless actually needed
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Find essential ROS2 dependencies
find_package(ament_cmake REQUIRED)

# ----- Remove this section unless you actually need the hello executable -----
# add_executable(hello src/hello.cpp)
# target_include_directories(hello PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(hello PUBLIC c_std_99 cxx_std_17)
# ----------------------------------------------------------------------------

# Add Gazebo model path using environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

# Install Gazebo resources (CORRECTED SECTION)
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Remove testing section unless actually needed
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Final ROS2 package declaration
ament_package()