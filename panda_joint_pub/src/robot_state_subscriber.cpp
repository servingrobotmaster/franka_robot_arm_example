#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateSubscriber : public rclcpp::Node
{
public:
  JointStateSubscriber()
    : Node("joint_state_subscriber")
  {
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "panda_link0/joint_states", 10, std::bind(&JointStateSubscriber::jointStateCallback, this, std::placeholders::_1));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {

     // 예를 들어, 현재 위치 정보를 출력
    RCLCPP_INFO(get_logger(), "Received joint states: %f %f %f %f %f %f %f",
                msg->position[0], msg->position[1], msg->position[2],
                msg->position[3], msg->position[4], msg->position[5], msg->position[6]);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}