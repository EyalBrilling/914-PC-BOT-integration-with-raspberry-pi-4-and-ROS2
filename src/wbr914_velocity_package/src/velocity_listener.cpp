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

void CmdVelListener::get_velocity_service(const std::shared_ptr<wbr914_velocity_package::srv::VelocityGet::Request> request,
          std::shared_ptr<wbr914_velocity_package::srv::VelocityGet::Response> response) {   

            int32_t left_vel, right_vel;

            if(wbr914.GetVelocityInTicks(&left_vel, &right_vel) < 0){
              RCLCPP_ERROR(this->get_logger(), "Couldn't get velocity from robot using GetVelocityInTicks");
              response->success = false;
              return;
            }

            /*
              Cast from wheels velocity in ticks to linear and angular velocity
              ****Notice this is a simplified model given by the company****
              If other model is found to be needed, implement it
              in another same-format cpp file and compile against it.
            */ 
            double lv = wbr914.Vel2MPS( left_vel );
            double rv = wbr914.Vel2MPS( right_vel );
            double trans_vel = (lv + rv)/2;
            double rot_vel = (lv - rv)/2;
            // DEFAULT_AXLE_LENGTH/2.0 is the radius of the robot wheels
            double rot_vel_rad = rot_vel/(DEFAULT_AXLE_LENGTH/2.0);

            response->success = true;
            response->response_twist.linear.x = trans_vel;
            response->response_twist.angular.x = rot_vel_rad;
            return;

          }

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelListener>());
  rclcpp::shutdown();
  return 0;
  
}