cmake_minimum_required(VERSION 3.8)
project(mavlink2roboclaw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboclaw REQUIRED)
find_package(mavros_msgs REQUIRED)

# specify include directories
include_directories(
    include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ssp_interfaces_INCLUDE_DIRS}
)

add_executable(mav2robo_single_node
    src/mav2robo_single_node.cpp)
ament_target_dependencies(mav2robo_single_node
    "rclcpp"
    "roboclaw"
    "mavros_msgs")

add_executable(mav2robo_diff_node
    src/mav2robo_diff_node.cpp)
ament_target_dependencies(mav2robo_diff_node
    "rclcpp"
    "roboclaw"
    "mavros_msgs")

add_executable(mav2robo_switch_node
    src/mav2robo_diff_node.cpp)
ament_target_dependencies(mav2robo_switch_node
    "rclcpp"
    "roboclaw"
    "mavros_msgs")

install(TARGETS
    mav2robo_single_node
    mav2robo_diff_node
    mav2robo_switch_node
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
