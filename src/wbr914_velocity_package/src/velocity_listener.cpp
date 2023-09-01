#include "velocity_listener.h"


void CmdVelListener::velocity_callback(const geometry_msgs::msg::Twist& msg){

  float trans = msg.linear.x;
  float rot = msg.angular.x * DEFAULT_AXLE_LENGTH/2.0;
  float l = trans - rot;
  float r = trans + rot;

  // Reduce the speed on each wheel to the max
  // speed to prevent the stepper motors from stalling
  // Scale the speed of the other wheel to keep the
  // turn of the same rate. Note the turn geometry
  // will be affected.
  if ( fabs( l ) > MOTOR_DEF_MAX_SPEED )
  {
    float dir = l/fabs(l);
    float scale = l/MOTOR_DEF_MAX_SPEED;
    l = dir*MOTOR_DEF_MAX_SPEED;
    r = r/fabs(scale);
  }

  if ( fabs( r ) > MOTOR_DEF_MAX_SPEED )
  {
    float dir = r/fabs(r);
    float scale = r/MOTOR_DEF_MAX_SPEED;
    r = dir*MOTOR_DEF_MAX_SPEED;
    l = l/fabs(scale);
  }

  int32_t leftvel = wbr914.MPS2Vel( l );
  int32_t rightvel = wbr914.MPS2Vel( r );

  wbr914.SetContourMode( VelocityContouringProfile );
    // now we set the speed
  if ( wbr914._motorsEnabled )
  {
    wbr914.SetVelocityInTicks(leftvel,rightvel); 
  }
  else
  {
    wbr914.SetVelocityInTicks( 0, 0 );
    printf( "Motors not enabled\n" );
  }
  wbr914.UpdateM3();
    
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelListener>());
  rclcpp::shutdown();
  return 0;
  
}