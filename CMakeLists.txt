cmake_minimum_required(VERSION 3.0)

project(MyProject)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add the executable target
add_executable(test_minimal wbr_main_try.cpp wbr914_minimal.cpp wbr914_minimal.h)

# Enable debugging symbols
target_compile_options(test_minimal PRIVATE -g)

# Link the necessary libraries (if needed)
# target_link_libraries(MyExecutable PRIVATE some_library_name)

# If you want to generate Makefiles for debugging (e.g., in Linux):
# cmake -DCMAKE_BUILD_TYPE=Debug .

# If you want to generate Makefiles for release (e.g., in Linux):
# cmake -DCMAKE_BUILD_TYPE=Release .


