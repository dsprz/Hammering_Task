cmake_minimum_required(VERSION 3.10)        # Minimum required CMake version
project(my_project)                         # Project name

# Find Pinocchio library
find_package(pinocchio REQUIRED)

# Add executable target, compile main.cpp
add_executable(gradient_ascent gradient_ascent.cpp)

# Link the Pinocchio library to your program
target_link_libraries(gradient_ascent pinocchio::pinocchio)