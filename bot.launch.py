from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    projectbot_pkg = get_package_share_directory('projectbot')
    urdf_path = os.path.join(projectbot_pkg, 'urdf', 'arm.urdf')
    controllers_yaml = os.path.join(projectbot_pkg, 'config', 'controllers.yaml')

    robot_description_a = {'robot_description': '<robot name="arm_a"><group name="arm_a">' + open(urdf_path).read() + '</group></robot>'}
    robot_description_b = {'robot_description': '<robot name="arm_b"><group name="arm_b">' + open(urdf_path).read() + '</group></robot>'}

    return [
        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            )
        ),

        # Robot A
        Node(
            package='projectbot',
            executable='spawn_bot',
            namespace='arm_a',
            arguments=['arm_a', '0.0', '0.0', '/arm_a', urdf_path, 'arm_a'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='arm_a',
            parameters=[robot_description_a],
            remappings=[('/joint_states', 'joint_states')]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='arm_a',
            parameters=[controllers_yaml],
            output='screen'
        ),
        Node(
            package='ros2_control',
            executable='spawner.py',
            namespace='arm_a',
            arguments=['arm_a/joint_state_broadcaster', '--controller-manager', '/arm_a/controller_manager'],
            output='screen',
        ),
        Node(
            package='ros2_control',
            executable='spawner.py',
            namespace='arm_a',
            arguments=['arm_a/joint_trajectory_controller', '--controller-manager', '/arm_a/controller_manager'],
            output='screen',
        ),

        # Robot B
        Node(
            package='projectbot',
            executable='spawn_bot',
            namespace='arm_b',
            arguments=['arm_b', '1.5', '0.0', '/arm_b', urdf_path, 'arm_b'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='arm_b',
            parameters=[robot_description_b],
            remappings=[('/joint_states', 'joint_states')]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='arm_b',
            parameters=[controllers_yaml],
            output='screen'
        ),
        Node(
            package='ros2_control',
            executable='spawner.py',
            namespace='arm_b',
            arguments=['arm_b/joint_state_broadcaster', '--controller-manager', '/arm_b/controller_manager'],
            output='screen',
        ),
        Node(
            package='ros2_control',
            executable='spawner.py',
            namespace='arm_b',
            arguments=['arm_b/joint_trajectory_controller', '--controller-manager', '/arm_b/controller_manager'],
            output='screen',
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
