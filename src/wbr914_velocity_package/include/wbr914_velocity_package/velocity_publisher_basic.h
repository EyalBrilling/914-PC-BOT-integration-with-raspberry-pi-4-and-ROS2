#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

class CmdVelPublisher_basic: public rclcpp::Node {
public:
  CmdVelPublisher_basic() : Node("CmdVelPublisher_basic") {
    // Create a publisher to the 'velocity' topic
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("velocity", 10);

    // Create a 'while' loop that will keep running until the user enters 'exit'
    while (true) {
      // Read the input from the user
      std::string input;
      std::cout << "Enter input: ";
      std::cin >> input;

      // If the input is not 'exit', publish 'hello' to the 'velocity' topic
      if (input != "exit") {
        std_msgs::msg::String message;
        message.data = "hello";
        publisher.publish(message);
      }
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

