import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # URDF 파일을 robot_description 매개변수로 설정
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/path/to/your/urdf/my_robot.urdf').read()}],
        ),
        # RViz 2 실행
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'path/to/your/rviz/config/file.rviz'],  # RViz 2 설정 파일을 지정
        ),
    ])