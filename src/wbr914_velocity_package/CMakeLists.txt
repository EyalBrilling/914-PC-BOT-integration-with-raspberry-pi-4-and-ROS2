cmake_minimum_required(VERSION 3.8)
project(wbr914_velocity_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies. Add more as needed.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Set the project path from the enviorment variable
set(WBR914_PROJECT_PATH $ENV{WBR914_PROJECT_PATH})

include_directories(include/wbr914_velocity_package)
include_directories(${WBR914_PROJECT_PATH}/wbr914_base_driver)

add_executable(listener0 src/velocity_ros_wrapper.cpp include/wbr914_velocity_package/velocity_ros_wrapper.h ${WBR914_PROJECT_PATH}/wbr914_base_driver/wbr914_minimal.h ${WBR914_PROJECT_PATH}/wbr914_base_driver/wbr914_minimal.cpp)
ament_target_dependencies(listener0 rclcpp std_msgs)



install(TARGETS listener0
  DESTINATION lib/${PROJECT_NAME})


ament_package()
