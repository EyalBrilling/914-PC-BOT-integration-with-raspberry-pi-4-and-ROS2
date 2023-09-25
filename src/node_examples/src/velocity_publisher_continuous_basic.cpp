#include "velocity_publisher_continuous_basic.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  CmdVelPublisher_continuous_basic my_publisher_node;

  // Spin until ROS is shutdown
  rclcpp::spin(std::make_shared<CmdVelPublisher_continuous_basic>());

  rclcpp::shutdown();

  return 0;
}
