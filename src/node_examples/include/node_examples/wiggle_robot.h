#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

const std::string velocity_topic_name = "velocity_cmd";

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
    geometry_msgs::msg::Twist msg;
    // Create a 'while' loop that wiggle the robot until node exits with ctrl+c
      // Read input from the user
      std::string input;
      std::vector< std::string > tokens;
      std::string token;
      printf("Enter wiggle time per side(secs), x linear velocity(m/sec) and x angular velocity(rads/sec): ");

      // Get input from user and save in a vector
      std::getline(std::cin, input);
      std::stringstream ss(input);
      while (ss >> token) {
        tokens.push_back(token);

      
      }

      // Expecting only x of linear and angular velocities
      if(tokens.size() != 3){
        printf("input error. 3 values are expected.\n");
        exit(0);
      }
        // Construct messages
        int wiggle_time = std::stod(tokens[0]);
        geometry_msgs::msg::Twist msgOneDirection;
        msgOneDirection.linear.x = std::stod(tokens[1]);
        msgOneDirection.angular.x = std::stod(tokens[2]);
        // Opposite velocity values
        geometry_msgs::msg::Twist msgSecondDirection;
        msgSecondDirection.linear.x = -std::stod(tokens[1]);
        msgSecondDirection.angular.x = -std::stod(tokens[2]);
        // Run infinitly,wiggling from side to side.
        while (rclcpp::ok()) {
        // Run the command for some seconds. Robot expects contiunally command
        auto startTime = std::chrono::steady_clock::now();
        auto endTime = startTime + std::chrono::seconds(wiggle_time);

        printf("Publishing for %i secs to topic %s Linear x: %f, Angular x: %f \n", wiggle_time, velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);
        msg = msgOneDirection;
        while (std::chrono::steady_clock::now() < endTime) {
            publisher->publish(msg);
        }
        auto startTime2 = std::chrono::steady_clock::now();
        auto endTime2 = startTime2 + std::chrono::seconds(wiggle_time);

        printf("Publishing other side movement\n");
        printf("Publishing for %i secs to topic %s Linear x: %f, Angular x: %f \n", wiggle_time, velocity_topic_name.c_str(), msg.linear.x, msg.angular.x);
        msg = msgSecondDirection;
        while (std::chrono::steady_clock::now() < endTime2) {
            publisher->publish(msgSecondDirection);
        }
            
        }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

