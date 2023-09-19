#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

const std::string velocity_topic_name = "velocity_cmd";
const int velocity_command_secs = 10;

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
      printf("Enter x linear velocity(m/sec) and x angular velocity(rads/sec) or exit to stop publisher: ");

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
        // Construct message
        geometry_msgs::msg::Twist msg;
        msg.linear.x = std::stod(tokens[0]);
        msg.angular.x = std::stod(tokens[1]);

        // Run the command for some seconds. Robot expects contiunally command
        auto startTime = std::chrono::steady_clock::now();
        auto endTime = startTime + std::chrono::seconds(velocity_command_secs);

        printf("Publishing for %i secs to topic %s Linear x: %f, Angular x: %f \n", velocity_command_secs, velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);

        while (std::chrono::steady_clock::now() < endTime) {
            publisher->publish(msg);
        }
            printf("Ended publishing command\n");
        }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

