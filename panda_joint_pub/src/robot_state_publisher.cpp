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
      //std::cout << "Published joint states:" << std::endl;
      joint_state_msg->position = {0.5, 0.3, 0.2, 0, 0, 0, 0};
      //     // Set velocity values (assuming a constant velocity for this example)
      joint_state_msg->velocity = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

     // Set effort values (assuming a constant effort for this example)
      joint_state_msg->effort = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
      joint_state_publisher->publish(*joint_state_msg);
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"

// class FrankaJointPublisher : public rclcpp::Node
// {
// public:
//   FrankaJointPublisher()
//     : Node("franka_joint_publisher")
//   {
//     // Create a publisher for JointState messages
//     joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("panda_link0/joint_states", 10);

//     // Create a timer to publish joint states periodically
//     timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishJointStates(); });
//   }

// private:
//   void publishJointStates()
//   {
//     auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

//     // Populate joint state message with joint values from your Franka arm
//     joint_state_msg->position = {0.0, 0.2, -0.5, 0.2, -0.1, 0.5, -0.3};
//     // Set velocity values (assuming a constant velocity for this example)
//     joint_state_msg->velocity = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

//     // Set effort values (assuming a constant effort for this example)
//     joint_state_msg->effort = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
//     // Set other fields if needed (velocity, effort, etc.)

//     // Publish the joint state message
//     joint_state_publisher_->publish(*joint_state_msg);
//   }

//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<FrankaJointPublisher>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
