from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    waypoint_node = Node(
        package='trajectory_control',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        output='screen'
    )

    smoother_node = Node(
        package='trajectory_control',
        executable='path_smoother',
        name='path_smoother',
        output='screen'
    )

    trajectory_node = Node(
        package='trajectory_control',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen'
    )

    controller_node = Node(
        package='trajectory_control',
        executable='controller',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        waypoint_node,
        smoother_node,
        trajectory_node,
        controller_node
    ])