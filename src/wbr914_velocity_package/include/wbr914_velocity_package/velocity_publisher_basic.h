#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

const std::string velocity_topic_name = "velocity";

class CmdVelPublisher_basic: public rclcpp::Node {
public:
  CmdVelPublisher_basic() : Node("CmdVelPublisher_basic") {
    // Create a publisher to the 'velocity' topic
    publisher = this->create_publisher<geometry_msgs::msg::Twist>(velocity_topic_name, 10);
    // Create a 'while' loop that will keep running until the user enters 'exit'
    while (rclcpp::ok()) {
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
        continue;
      }

        geometry_msgs::msg::Twist msg;
        msg.linear.x = std::stod(tokens[0]);
        msg.angular.x = std::stod(tokens[1]);
        publisher->publish(msg);
        printf(" Published to topic %s Linear x: %f, Angular x: %f \n", velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};
