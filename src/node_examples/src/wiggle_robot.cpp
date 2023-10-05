#include "wiggle_robot.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Spin until ROS is shutdown
  rclcpp::spin(std::make_shared<Wiggle_robot>());

  rclcpp::shutdown();

  return 0;
}
