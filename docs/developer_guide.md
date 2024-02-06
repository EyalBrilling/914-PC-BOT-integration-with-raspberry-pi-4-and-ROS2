# Developer guide

Look at the package node_examples for nodes that communicate with the robot.

It is highly recommanded to use the [ros2 guides](https://docs.ros.org/en/iron/Tutorials.html) and learn from them

## Overview on communication

The pi4 communicates with the M3 I/O and
motor control board over a serial-to-USB driver. The serial commands are
used to communicate with two PMD motion control chips that drive the
stepper motors and control the onboard I/O.

## Using the wbr914 - program logic

### Directly calling the driver functions

Let's look at this simple example of directly calling the driver functions to move the robot:

```C++
// Including the header of the driver from which the functions are called.
#include "wbr914_minimal.h"

int main(){
    // The driver is implemented in a class wbr914_minimal. when we need a function it always gets called from the class itself.
    wbr914_minimal wbr914;

    /*
    This is the first step everytime the robot gets turned on.
    It takes care of everything related to the serial port and variables saved on the M3.
    1) Gets serial port information, needs to be known so we can reset it when we turn the robot off.(in deconstructor)
    2) setups variables for communication speed and such
    3) Reset variables on M3 like odometry data
    */
    int mainSetupFlag = wbr914.MainSetup();
    if(mainSetupFlag==0){
      printf("wbr914 MainSetup succeed\n");
    }
    /*
        Enable the motors. You should hear a click.
    */
    bool enableMotorsFlag = wbr914.EnableMotors(true);
    if(enableMotorsFlag==true){
      printf("enableMotors is true\n");
    }
    /*
    UpdateM3 motors.
    To be called when we talk with the motors from some function. which are all functions that send commands to LEFT_MOTOR and RIGHT_MOTOR codes.
    For example : sendCmd16( LEFT_MOTOR, SETPROFILEMODE, prof, 2, ret)<0)
    */
    wbr914.UpdateM3();
    
    /*
      Tell M3 how fast de/acceleration should be.
      Read more about profiles here:
      https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article
      https://www.motioncontroltips.com/what-is-a-motion-profile/
      There are 3 profiles(enum ProfileMode_t) but the original developers always use VelocityContouringProfile when handling velocity commands. So it is an option to use the other profiles in M3 but we use VelocityContouringProfile for now.
    */
    wbr914.SetContourMode( VelocityContouringProfile );

    /* 
      Loop to send commands of velocity
    */
    for(int i=0;i<=1000;i++){
    wbr914.SetVelocityInTicks(10000,5000);
    wbr914.UpdateM3();
    }
    /*
    Closes connecting to serial port. Gets called from deconstructor too.
    */
    wbr914.MainQuit();
}
```


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
