cmake_minimum_required(VERSION 3.8)
project(clothoid_motion_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include the header files from the include/ directory
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_executable(clot_motion_planner_node
    src/clothoid_path.cpp
    src/clothoid_motion_planner.cpp
    src/clot_motion_planner_node.cpp)

# Link Eigen
target_link_libraries(clot_motion_planner_node Eigen3::Eigen)

ament_target_dependencies(clot_motion_planner_node 
    rclcpp
    std_msgs
    custom_interfaces
    ament_index_cpp)

install(TARGETS
  clot_motion_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

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
