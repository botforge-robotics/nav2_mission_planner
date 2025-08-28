from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


# Launch Arguments
ARGUMENTS = [
    DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Raw camera image topic to subscribe to'
    ),
]


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    camera_topic = LaunchConfiguration('camera_topic')

    # ROS Bridge for WebSocket communication
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[
            {'send_action_goals_in_new_thread': True},
            {'call_services_in_new_thread': True},
        ],
        output='screen'
    )

    # ROS API node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    # Web Video Server with custom parameters
    web_video_server_node = Node(
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
    )

    # Image republisher: raw -> compressed camera preview
    republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='rgb_compressed_republisher',
        arguments=[
            'raw',
            'in:=' + camera_topic.perform(context),
            '_image_transport:=compressed'
        ],
        parameters=[
            {'jpeg_quality': 50}
        ],
        output='screen'
    )

    # Add Launch Manager Node
    launch_manager_node = Node(
        package='nav2_mission_planner',
        executable='launch_manager',
        name='launch_manager',
        output='screen',
        emulate_tty=True
    )

    return [
        rosbridge_node,
        rosapi_node,
        web_video_server_node,
        republisher_node,
        launch_manager_node
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
