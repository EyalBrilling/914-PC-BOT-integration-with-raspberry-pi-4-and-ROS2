#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

const std::string velocity_topic_name = "velocity";
const int velocity_command_secs = 10;

/*
  Publishing Twist message indifindlly while getting user input to change velocity to be sent.
  New thread is in charge of sending message - only reading from msg.
  main thread is in charge of getting input from user. writing to msg.
*/
class CmdVelPublisher_continuous_basic: public rclcpp::Node {
public:
  CmdVelPublisher_basic() : Node("CmdVelPublisher_continuous_basic") {
    // Create a publisher to the 'velocity' topic
    publisher = this->create_publisher<geometry_msgs::msg::Twist>(velocity_topic_name, 10);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.angular.x = 0;

    while (rclcpp::ok()) {
      // Open new thread to continually publish Twist msg.
      std::thread([](){
        while(rclcpp::ok()){
          publisher->publish(msg);
        }
      });
      // Read input from the user
      std::string input;
      std::vector< std::string > tokens;
      std::string token;
      printf("Enter x linear velocity and x angular velocity or exit to stop publisher: ");

      // Get input from user and save in a vector
      std::getline(std::cin, input);
      std::stringstream ss(input);
      while (ss >> token) {
        tokens.push_back(token);

      
      }
      if(input == "exit"){
        rclcpp::shutdown();
        break;
      }

      // Expecting only x of linear and angular velocities
      if(tokens.size() != 2){
          printf("input error. 2 values are expected.\n");
          // For safety,in case of input error reset movement
          msg.linear.x = 0;
          msg.angular.x = 0;
          continue;
      }
        // Construct message.
        msg.linear.x = std::stod(tokens[0]);
        msg.angular.x = std::stod(tokens[1]);


        printf("Publishing to topic %s Linear x: %f, Angular x: %f \n", velocity_command_secs, velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);
        }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

