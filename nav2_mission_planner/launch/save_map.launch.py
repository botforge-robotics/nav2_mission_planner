from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory, PackageNotFoundError
from pathlib import Path
import os


def check_path(context):
    map_path = context.launch_configurations['map_path']
    map_name = context.launch_configurations['map_name']

    try:
        # Split package and path
        if '/' not in map_path:
            raise RuntimeError(
                f"Invalid map_path format: {map_path}. Use 'package_name/path'")

        package_name, rel_path = map_path.split('/', 1)

        # Verify package exists
        try:
            pkg_share = get_package_share_directory(package_name)
        except PackageNotFoundError:
            raise RuntimeError(f"Package '{package_name}' not found")

        # Create full path
        full_path = Path(pkg_share) / rel_path
        full_path.mkdir(parents=True, exist_ok=True)

        # Verify write permissions
        if not os.access(str(full_path), os.W_OK):
            raise RuntimeError(
                f"No write permissions for directory: {full_path}")

        return [ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', PathJoinSubstitution([
                    FindPackageShare(package_name),
                    rel_path,
                    map_name
                ])
            ],
            output='screen'
        )]

    except Exception as e:
        raise RuntimeError(f"Map save failed: {str(e)}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='my_map',
            description='Base name for map files'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value='nav2_mission_planner/maps',
            description='Path to save maps in format "package_name/folder/subfolder"'
        ),
        OpaqueFunction(function=check_path)
    ])
