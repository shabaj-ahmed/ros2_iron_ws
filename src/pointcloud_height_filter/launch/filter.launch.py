from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    min_height = LaunchConfiguration('min_height', default='0.0')
    max_height = LaunchConfiguration('max_height', default='2.0')
    input_topic = LaunchConfiguration('input', default='/points_in')
    output_topic = LaunchConfiguration('output', default='/points_filtered')

    return LaunchDescription([
        DeclareLaunchArgument('min_height', default_value='0.0'),
        DeclareLaunchArgument('max_height', default_value='2.0'),
        DeclareLaunchArgument('input', default_value='/points_in'),
        DeclareLaunchArgument('output', default_value='/points_filtered'),

        Node(
            package='pointcloud_height_filter',
            executable='height_filter_node',
            name='height_filter',
            output='screen',
            parameters=[{
                'min_height': min_height,
                'max_height': max_height,
            }],
            remappings=[
                ('input',  input_topic),
                ('output', output_topic),
            ],
        ),
    ])
