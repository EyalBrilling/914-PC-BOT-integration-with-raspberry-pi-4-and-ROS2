#include "velocity_ros_wrapper.h"


CmdVelListener::CmdVelListener() {
  // Initialize ROS.
  ros::init(argc, argv, "cmd_vel_listener");

  wbr914.MainSetup();
  wbr914.UpdateM3();
  bool test = wbr914.EnableMotors(true);  
  wbr914.UpdateM3();
  
  // Create a node.
  ros::NodeHandle nh;

  // Create a subscriber.
  sub = nh.subscribe("/cmd_vel", 1, &CmdVelListener::OnTwist, this);
}


CmdVelListener::~CmdVelListener(){
  wbr914.MainQuit();
}

void CmdVelListener::OnCmd(const geometry_msgs::Twist& twist){
  wbr914.SetContourMode( VelocityContouringProfile );
  for(int i=0;i<=1000;i++){
  wbr914.SetVelocityInTicks(40000,40000);
  wbr914.UpdateM3();
    }
}

int main(){

  
}