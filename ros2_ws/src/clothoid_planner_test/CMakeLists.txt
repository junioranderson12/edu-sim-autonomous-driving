cmake_minimum_required(VERSION 3.5)
project(clothoid_planner_test)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the header files from the include/ directory
include_directories(include)

# Add an executable from your source file(s)
add_executable(main
    src/main.cpp
    src/clothoid_path.cpp
)