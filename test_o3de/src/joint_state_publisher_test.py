#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState

def main():
    rclpy.init()

    node = rclpy.create_node('panda_joint_states_publisher')

    # JointState 메시지 생성
    joint_state_msg = JointState()
    joint_state_msg.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Publisher 생성
    publisher = node.create_publisher(JointState, '/o3de_ros2_node/panda/joint_states', 10)

    # 일정 간격으로 JointState 메시지를 발행
    timer_period = 1.0  # 초 단위
    timer = node.create_timer(timer_period, lambda: publish_joint_states(publisher, joint_state_msg))

    rclpy.spin(node)

def publish_joint_states(publisher, joint_state_msg):
    joint_state_msg.header.stamp = rclpy.time.Time().to_msg()  # 현재 시간으로 업데이트
    publisher.publish(joint_state_msg)

if __name__ == '__main__':
    main()