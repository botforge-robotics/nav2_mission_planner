from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ROS Bridge for WebSocket communication
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[
                {'send_action_goals_in_new_thread': True},
                {'call_services_in_new_thread': True},
            ],
            output='screen'
        ),

        # ROS API node
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'
        ),

        # Web Video Server with custom parameters
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[
                {'port': 8081},
                {'address': '0.0.0.0'},
                {'server_threads': 4},
                {'default_stream_type': 'ros_compressed'}
            ],
            output='screen'
        ),

        # Image republisher: raw -> compressed camera preview
        Node(
            package='image_transport',
            executable='republish',
            name='rgb_compressed_republisher',
            arguments=[
                'raw',
                'in:=/oakd/rgb/preview/image_raw',
                '_image_transport:=compressed'
            ],
            parameters=[
                {'jpeg_quality': 50}
            ],
            output='screen'
        ),

        # Add Launch Manager Node
        Node(
            package='nav2_mission_planner',
            executable='launch_manager',
            name='launch_manager',
            output='screen',
            emulate_tty=True
        ),
    ])
