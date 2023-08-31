#include "velocity_publisher_basic.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  MyPublisherNode my_publisher_node;

  // Spin until ROS is shutdown
  rclcpp::spin();

  rclcpp::shutdown();

  return 0;
}
