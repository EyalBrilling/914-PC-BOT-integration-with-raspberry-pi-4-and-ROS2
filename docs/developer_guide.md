# Developer guide

Look at the package node_examples for nodes that communicate with the robot.

It is highly recommanded to use the [ros2 guides](https://docs.ros.org/en/iron/Tutorials.html) and learn from them

## Creating new nodes

Every new node requires some similar steps:

1) Create a header file
2) Create a cpp file
3) CMakeLists configuration

### Node Header file

```cpp
// Include ros
#include <rclcpp/rclcpp.hpp>
/*
 Include any msgs/services from other packages.
*/ 
// For example,twist is a ros2 msg from the geometry_msgs package
#include "geometry_msgs/msg/twist.hpp"
// For example,the custom service in wbr914_package
#include "wbr914_package/srv/velocity_get.hpp"

// Create a class that inherits ros2 node
class class_name: public rclcpp::Node {
public:

    // Constructor 
   class_name() : Node("your_node_name") {
    /*
     Create the node publishers/clients in the constructor.
    */
    // For example, a publisher to the "velocity_cmd" topic.
      publisher = this->create_publisher<geometry_msgs::msg::Twist>("velocity_cmd", 10);
    
  }

    Class function declrations(Can also be private)
    .
    .
    .
  private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

}
```

### Node CPP file

```cpp
#include "<header_file_name.h>"

Class function implementations
.
.
.

int main(int argc, char** argv) {
  // Init ros2(it is done for each node,ros2 handles it)
  rclcpp::init(argc, argv);

  // Spin until ROS is shutdown
  rclcpp::spin(std::make_shared<class_name>());

  rclcpp::shutdown();

  return 0;
}
```

### Configure CMakeLists

#### Add executable

Create an executable file for the node:

```cmake
add_executable(executable_name
src/cpp_file_name.cpp
include/<package_name>/header_file_name.h) 
```

#### Add ros2 dependecies

ament is the build system used in ros2.  
We tell it which ros2 packages are needed for the executable:

```cmake
ament_target_dependencies(executable_name rclcpp geometry_msgs wbr914_package)
```

rclcpp is always needed.  
The rest depends on your project.  
Notice any package used here must be found in the CMakeLists by the line:

```cmake
find_package(package_name REQUIRED)
```

Use find_package() before creating the executable.

#### Install target

In the end of the CMakeLists file, install of the executables is done:

```cmake
install(TARGETS
 executable_name
  DESTINATION lib/${PROJECT_NAME})
```

## Working on the robot driver

The new driver in [/wbr914_base_driver/](/wbr914_base_driver/)  doesn't implement all of the original robot driver functions in [/wbr914_base_driver/player_driver/](/wbr914_base_driver/player_driver/).  
All player structs and functions used are gone.  

There are a few functions that can be added in the future to the new driver based on the player driver, noticablly IR and analog functions that the robot gives but right now aren't used.

If you want to implement those, in the cpp file of the old driver there is documntation for each function - if it is used or not in the new driver.
