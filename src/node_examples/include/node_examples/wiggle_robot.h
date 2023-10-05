#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

const std::string velocity_topic_name = "velocity_cmd";
const int velocity_command_secs = 10;

/*
  Publishing Twist message indifindlly while getting user input to change velocity to be sent.
  New thread is in charge of sending message - only reading from msg.
  main thread is in charge of getting input from user. writing to msg.
*/
class Wiggle_robot: public rclcpp::Node {
public:
  Wiggle_robot() : Node("Wiggle_robot") {
      // Create a publisher to the 'velocity' topic
    publisher = this->create_publisher<geometry_msgs::msg::Twist>(velocity_topic_name, 10);
    // Create a 'while' loop that wiggle the robot until node exits with ctrl+c
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

      // Expecting only x of linear and angular velocities
      if(tokens.size() != 2){
        printf("input error. 2 values are expected.\n");
        exit(0);
      }
        // Construct message
        geometry_msgs::msg::Twist msg;
        msg.linear.x = std::stod(tokens[0]);
        msg.angular.x = std::stod(tokens[1]);
        // Run infinitly,wiggling from side to side.
        while (rclcpp::ok()) {
        // Run the command for some seconds. Robot expects contiunally command
        auto startTime = std::chrono::steady_clock::now();
        auto endTime = startTime + std::chrono::seconds(velocity_command_secs);

        printf("Publishing for %i secs to topic %s Linear x: %f, Angular x: %f \n", velocity_command_secs, velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);

        while (std::chrono::steady_clock::now() < endTime) {
            publisher->publish(msg);
        }
        startTime = std::chrono::steady_clock::now();
        endTime = startTime + std::chrono::seconds(velocity_command_secs);

        msg.angular.x = -msg.angular.x;
        printf("Publishing other side movement\n");
        printf("Publishing for %i secs to topic %s Linear x: %f, Angular x: %f \n", velocity_command_secs, velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);
        while (std::chrono::steady_clock::now() < endTime) {
            publisher->publish(msg);
        }
            
        }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

