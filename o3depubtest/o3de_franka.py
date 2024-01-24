import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os

def main():
    rclpy.init()

    node = rclpy.create_node('spawn_entity_node')

    # Wait for the service to be available
    node.get_logger().info('Waiting for Gazebo SpawnEntity service...')
    spawn_entity_client = node.create_client(SpawnEntity, '/spawn_entity')

    while not spawn_entity_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Gazebo SpawnEntity service not available, waiting...')

    node.get_logger().info('Gazebo SpawnEntity service is available.')

    # Load URDF file content
    urdf_file_path = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    with open(urdf_file_path, 'r') as file:
        urdf_content = file.read()

    # Create a SpawnEntity request
    spawn_entity_request = SpawnEntity.Request()
    spawn_entity_request.name = 'panda'  # 엔터티의 이름 설정
    spawn_entity_request.xml = urdf_content  # URDF 파일의 내용을 문자열로 설정
    spawn_entity_request.robot_namespace = 'panda'  # 로봇 네임스페이스 설정
    spawn_entity_request.initial_pose = Pose('line1')  # 엔터티의 초기 위치 설정

    # Call the SpawnEntity service
    spawn_entity_future = spawn_entity_client.call_async(spawn_entity_request)
    rclpy.spin_until_future_complete(node, spawn_entity_future)

    if spawn_entity_future.result() is not None:
        node.get_logger().info('SpawnEntity service call successful.')
    else:
        node.get_logger().error('Failed to call SpawnEntity service.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()