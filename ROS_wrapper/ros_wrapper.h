#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <map>
#include <string>
#include <vector>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include "player_driver/wbr914.h"

class MotorDriverROSWrapper 
{
private:
    std::unique_ptr<wbr914> wbr914Controller;
    ros::Subscriber speed_command_subscriber_;
    ros::ServiceServer stop_motor_server_;
    int max_wheel_speed = MAX_WHEELSPEED;
public:
    MotorDriverROSWrapper(ros::NodeHandle *nh,ConfigFile* cf, int section)
    {
        wbr914Controller.reset(new wbr914Controller(cf,section));

        speed_command_subscriber_ = nh->subscribe(
            "speed_command", 10, &MotorDriverROSWrapper::callbackSpeedCommand, this);

        stop_motor_server_ = nh->advertiseService(
            "stop_motor", &MotorDriverROSWrapper::callbackStop, this);
    }


    bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        stop();
        res.success = true;
        res.message = "Successfully stopped motor.";
        return true;
    }

     void callbackSpeedCommand(const std_msgs::Float32 &mpsL,const std_msgs::Float32 &mpsR)
    {
        wbr914Controller->SetVelocity(mpsL.data,mpsR.data);
    }
    void stop()
    {
        wbr914Controller->StopRobot();
    }
};