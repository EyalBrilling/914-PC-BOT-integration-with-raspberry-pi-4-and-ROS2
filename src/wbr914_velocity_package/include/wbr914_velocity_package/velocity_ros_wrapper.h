#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "wbr914_minimal.h"

class CmdVelListener : public rclcpp::Node{
public:
  CmdVelListener();
  ~CmdVelListener();
/*
 Twist message is expected in the cmd_vel topic
 linear:
   x: 2.0
   y: 0.0
   z: 0.0
 angular:
   x: 0.0
   y: 0.0
   z: 0.0
*/
  void OnCmd(const geometry_msgs::Twist& twist);

private:
  ros::Subscriber sub;
  wbr914_minimal wbr914;
};