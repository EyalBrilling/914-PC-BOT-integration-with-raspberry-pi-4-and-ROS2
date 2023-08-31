#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "wbr914_minimal.h"

using std::placeholders::_1;

/*
  A ROS listener(subscriber) reading Twist messages from the 'velocity' topic.
  Only the x value of the linear and angular velocity is used.
*/
class CmdVelListener : public rclcpp::Node{
public:
  CmdVelListener() : Node("vel_listener_node"){
    // Open connection with wbr914 usb and enable motors
    wbr914.MainSetup();
    wbr914.UpdateM3();
    wbr914.EnableMotors(true);  
    wbr914.UpdateM3();

    // Create the subscriber to a 'velocity' topic.
    // Using placeholder _1 for keeping the option of using a ptr to Twist.
    sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "velocity",10,std::bind(&CmdVelListener::velocity_callback,this,_1));
  };

  ~CmdVelListener(){
    // Shutdown communication with wbr914
    wbr914.MainQuit();
  };

private:
/*
 Twist message is expected in the cmd_vel topic
 linear:
   x: X
   y: 0.0
   z: 0.0
 angular:
   x: X
   y: 0.0
   z: 0.0
*/
  void velocity_callback(const geometry_msgs::msg::Twist& msg);


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  wbr914_minimal wbr914;
};