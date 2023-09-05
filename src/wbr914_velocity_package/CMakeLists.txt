cmake_minimum_required(VERSION 3.8)
project(wbr914_velocity_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies. Add more as needed.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Set the project path from the enviorment variable
set(WBR914_PROJECT_PATH $ENV{WBR914_PROJECT_PATH})
message(STATUS "WBR914_PROJECT_PATH: ${WBR914_PROJECT_PATH}")

include_directories(
include/wbr914_velocity_package
${WBR914_PROJECT_PATH}/wbr914_base_driver
)


add_executable(wbr914_velocity_listener src/velocity_listener.cpp include/wbr914_velocity_package/velocity_listener.h ${WBR914_PROJECT_PATH}/wbr914_base_driver/wbr914_minimal.h ${WBR914_PROJECT_PATH}/wbr914_base_driver/wbr914_minimal.cpp)
add_executable(wbr914_velocity_publisher_basic src/velocity_publisher_basic.cpp include/wbr914_velocity_package/velocity_publisher_basic.h) 
add_executable(wbr914_velocity_publisher_continuous_basic src/velocity_publisher_continuous_basic.cpp include/velocity_publisher_continuous_basic.h) 

# Add needed ROS packages to the executables
ament_target_dependencies(wbr914_velocity_listener rclcpp geometry_msgs)
ament_target_dependencies(wbr914_velocity_publisher_basic rclcpp geometry_msgs)
ament_target_dependencies(wbr914_velocity_publisher_continuous_basic rclcpp geometry_msgs)

install(TARGETS
 wbr914_velocity_listener
 wbr914_velocity_publisher_basic
 wbr914_velocity_publisher_continuous_basic
  DESTINATION lib/${PROJECT_NAME})


ament_package()
