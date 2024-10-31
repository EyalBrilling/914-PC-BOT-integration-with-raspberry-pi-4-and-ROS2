#include "wbr914_node.h"


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

void CmdVelListener::get_velocity_service(const std::shared_ptr<wbr914_package::srv::VelocityGet::Request> request,
          std::shared_ptr<wbr914_package::srv::VelocityGet::Response> response) {   

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

void CmdVelListener::get_position_service(const std::shared_ptr<wbr914_package::srv::PositionGet::Request> request,
          std::shared_ptr<wbr914_package::srv::PositionGet::Response> response){
            int32_t left_pos = -57;
            int32_t right_pos = -57;
            const double TWOPI = 2.0*M_PI;
            if(wbr914.GetPositionInTicks(&left_pos,&right_pos) < 0){
            RCLCPP_ERROR(this->get_logger(), "Couldn't get position from robot using GetPositionInTicks");
            response->success = false;
            return;
            }

            // Calculate new position based on previous position and update it
            int32_t change_left  = left_pos - wbr914.last_lpos;
            int32_t change_right = right_pos - wbr914.last_rpos;
            wbr914.last_lpos = left_pos;
            wbr914.last_rpos = right_pos;

            // Calculate translational and rotational change
            // translational change = avg of both changes
            // rotational change is half the diff between both changes
            double transchange = wbr914.Ticks2Meters( (change_left + change_right)>>1 );
            double rotchange = wbr914.Ticks2Meters( (change_left - change_right)>>1 );

            // calc total yaw, constraining from 0 to 2pi
            wbr914._yaw += rotchange/(DEFAULT_AXLE_LENGTH/2.0);
            if ( wbr914._yaw < 0 )
              wbr914._yaw += TWOPI;
          
            if ( wbr914._yaw > TWOPI )
              wbr914._yaw -= TWOPI;

            // calc current x and y position
            wbr914._x += ( transchange * cos( wbr914._yaw ));
            wbr914._y += ( transchange * sin( wbr914._yaw ));

            response->success = true;
            response->response_pose.position.x = wbr914._x;
            response->response_pose.position.y = wbr914._y;
            response->response_pose.orientation.x = wbr914._yaw;
            return;
          }

void CmdVelListener::get_irdata_service(const std::shared_ptr<wbr914_package::srv::IrdataGet::Request> request,
          std::shared_ptr<wbr914_package::srv::IrdataGet::Response> response)
          {
            float voltages[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            float ranges[8] = {0, 0, 0, 0, 0, 0, 0, 0};

            if (wbr914.GetIRData(voltages, ranges)==0)
            {
              response->data_ir1 = voltages[0];  
              response->data_ir2 = voltages[1];
              response->data_ir3 = voltages[2];
              response->data_ir4 = voltages[3];
              response->data_ir5 = voltages[4];
              response->data_ir6 = voltages[5];
              response->data_ir7 = voltages[6];
              response->data_ir8 = voltages[7];
              response->range_ir1 = ranges[0];
              response->range_ir2 = ranges[1];
              response->range_ir3 = ranges[2];
              response->range_ir4 = ranges[3];
              response->range_ir5 = ranges[4];
              response->range_ir6 = ranges[5];
              response->range_ir7 = ranges[6];
              response->range_ir8 = ranges[7];
              response->success = true;
            }
            else
            {
              RCLCPP_ERROR(this->get_logger(), "Couldn't get IR date from robot using GetIRData");
              response->success = false;
              return;
            }
          }

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelListener>());
  rclcpp::shutdown();
  return 0;
  
}