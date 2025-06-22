cmake_minimum_required(VERSION 3.5)
project(clothoid_planner_test)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the header files from the include/ directory
include_directories(include)

find_package(Eigen3 REQUIRED)

# Add an executable from your source file(s)
add_executable(main
    src/main.cpp
    src/clothoid_path.cpp
)

target_link_libraries(main Eigen3::Eigen)