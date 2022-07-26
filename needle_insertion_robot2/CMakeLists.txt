cmake_minimum_required(VERSION 3.5)
project(needle_insertion_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# setup custom libraries
link_directories(${PROJECT_SOURCE_DIR}/lib)

# setup node executable
add_executable(insertion_robot_node src/insertion_robot_node.cpp src/NeedleInsertionRobot.cpp src/GalilController.cpp)
target_link_libraries(insertion_robot_node gclib gclibo)
target_include_directories(insertion_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(insertion_robot_node PUBLIC include/gclib) 
ament_target_dependencies(
    insertion_robot_node
    "rclcpp"
    "std_msgs"
    "geometry_msgs"
    "std_srvs"
    "rcl_interfaces"
)

install(TARGETS insertion_robot_node
  DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(
#  include
# )

ament_package()
