#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_state_publisher");

  // Setting QoS policy to keep the last 10 messages with volatile durability
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // Specify the correct topic name "panda_link0/joint_states"
  auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("panda_link0/joint_states", qos);

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [joint_state_publisher]() {
      auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
      // Populate joint_state_msg with joint values from your robot
      std::cout << "Published joint states:" << std::endl;
      joint_state_msg->position = {0.5, 0.05, 0.005, 0.03, 0.02, -0.05, 0.07, 0.03, 0.03};
      joint_state_publisher->publish(*joint_state_msg);
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}