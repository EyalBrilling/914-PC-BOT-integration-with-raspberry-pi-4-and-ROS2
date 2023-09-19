#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "wbr914_minimal.h"
#include <wbr914_velocity_package/srv/velocity_get.hpp>
#include <wbr914_velocity_package/srv/position_get.hpp>

using std::placeholders::_1;
using std::placeholders::_2;


/*
  A ROS listener(subscriber) reading Twist messages from the 'velocity' topic.
  Only the x value of the linear and angular velocity is used.
*/
class CmdVelListener : public rclcpp::Node{
public:
  CmdVelListener() : Node("vel_listener_node"){
    // Open connection with wbr914 usb,ready all needed settings and enable motors
    int mainSetupFlag = wbr914.MainSetup();
    if(mainSetupFlag==0){
      printf("wbr914 MainSetup succeed\n");
    }
    wbr914.UpdateM3();
    bool enableMotorsFlag = wbr914.EnableMotors(true);
    if(enableMotorsFlag==true){
      printf("enableMotors is true\n");
    }
    
    wbr914.UpdateM3();
    // Create the subscriber to a 'velocity' topic.
    // Using placeholder _1 for keeping the option of using a ptr to Twist.
    sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "velocity",10,std::bind(&CmdVelListener::velocity_callback,this,_1));

    // Create the service that will return velocity(Twist message) on request
    velocityGetService = this-> create_service<wbr914_velocity_package::srv::VelocityGet>("velocity_get_robot",
    std::bind(&CmdVelListener::get_velocity_service,this,_1,_2));

    // Create the service that will return position(Pose message) on request
    positionGetService = this-> create_service<wbr914_velocity_package::srv::PositionGet>("position_get_robot",
    std::bind(&CmdVelListener::get_position_service,this,_1,_2));
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

  void get_velocity_service(const std::shared_ptr<wbr914_velocity_package::srv::VelocityGet::Request> request,
        std::shared_ptr<wbr914_velocity_package::srv::VelocityGet::Response> response);

/*
 geometry_msgs/Pose message is returned by the get_position service
 point:
  float64 x: X
  float64 y: Y
  float64 z: 0.0

 orientation:
  float64 x: X
  float64 y: 0.0
  float64 z: 0.0
  float64 w: 0.0

  */

  void get_position_service(const std::shared_ptr<wbr914_velocity_package::srv::PositionGet::Request> request,
        std::shared_ptr<wbr914_velocity_package::srv::PositionGet::Response> response);
    

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  rclcpp::Service<wbr914_velocity_package::srv::VelocityGet>::SharedPtr velocityGetService;
  rclcpp::Service<wbr914_velocity_package::srv::PositionGet>::SharedPtr positionGetService;
  wbr914_minimal wbr914;
};