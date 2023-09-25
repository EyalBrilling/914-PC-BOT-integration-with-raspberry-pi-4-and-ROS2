#include "velocity_publisher_basic.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  CmdVelPublisher_basic my_publisher_node;

  // Spin until ROS is shutdown
  rclcpp::spin(std::make_shared<CmdVelPublisher_basic>());

  rclcpp::shutdown();

  return 0;
}
