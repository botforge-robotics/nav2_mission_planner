<p align="center">
    <img src="https://img.shields.io/badge/ROS2-jazzy-blue" alt="ROS2"/>
    <img src="https://img.shields.io/badge/license-MIT-blue" alt="License"/>
    <img src="https://img.shields.io/badge/platform-Ubuntu%2024.04-orange" alt="Platform"/>
    <img src="https://img.shields.io/github/stars/botforge-robotics/nav2_mission_planner?style=social" alt="Stars"/>
    <img src="https://img.shields.io/github/forks/botforge-robotics/nav2_mission_planner" alt="Forks"/>
    <img src="https://img.shields.io/github/issues/botforge-robotics/nav2_mission_planner" alt="Issues"/>
    <img src="https://img.shields.io/github/repo-size/botforge-robotics/nav2_mission_planner" alt="Repo Size"/>
</p>

<div align="center">
  <img src="./images/nmpLogo.png" width="80" alt="Logo">
</div>

<h1 align="center">Nav2 Mission Planner</h1>

## Table of Contents

- [Table of Contents](#table-of-contents)
- [📋 Overview](#-overview)
- [📱 App Showcase](#-app-showcase)
- [✅ Requirements](#-requirements)
- [⚙️ Configuration](#️-configuration)
  - [📋 Prerequisites](#-prerequisites)
  - [🔧 Required Wrapper Launch Files](#-required-wrapper-launch-files)
- [🚀 Installation](#-installation)
  - [Permanent Workspace Setup](#permanent-workspace-setup)
  - [Install Mission Planner App](#install-mission-planner-app)
- [📄 License](#-license)

---

## 📋 Overview

Companion package for Nav2 Mission Planner App.

---

## 📱 App Showcase

<p align="center">
  <a href="./images/mockup1.jpg">
    <img src="./images/mockup1.jpg" width="45%" alt="Nav2 Mission Planner Screenshot 1" style="border-radius:10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1)"/>
  </a>
  <a href="./images/mockup2.jpg">
    <img src="./images/mockup2.jpg" width="45%" alt="Nav2 Mission Planner Screenshot 2" style="border-radius:10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1)"/>
  </a>
</p>
<p align="center">
  <a href="./images/mockup3.jpg">
    <img src="./images/mockup3.jpg" width="45%" alt="Nav2 Mission Planner Screenshot 3" style="border-radius:10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1)"/>
  </a>
</p>

---

## ✅ Requirements

1. Robot with nav2 stack(Ros2 Jazzy or later) which is already able to navigate.
2. Nav2 Mission Planner App

---

## ⚙️ Configuration

### 📋 Prerequisites

1. Your robot is powered on and all **sensor drivers** are running:
   - 2D LiDAR publishing to `/scan`.
   - Camera streams (optional) publishing to `/image_raw`.
   - Odometry publishing to `/odom`.
   - A velocity command topic `/cmd_vel` for tele-operation or autonomous control.
2. The Nav2 stack is already up and running (e.g. launched via `nav2_bringup`):

   The robot must already be running **Nav2** from the `nav2_bringup` package _with your own navigation parameter file_. In practice this means that `navigation_launch.py` (or your own wrapper launch file) is active and was started like so:

   ```bash
   ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/your/nav2_params.yaml
   ```

   The YAML file should contain your robot-specific costmaps, planners, behaviour trees, etc.
   You can execute the command above directly, or embed the same include in a larger bring-up launch file so that Nav2 starts automatically on boot.

### 🔧 Required Wrapper Launch Files

<details open>
<summary><b>Launch File Requirements</b></summary>
<br>

To switch between mapping and localization modes, you need **two** minimal wrapper launch files in any ROS 2 package:

| Launch file              | Role           | Implementation Details                                                                             |
| ------------------------ | -------------- | -------------------------------------------------------------------------------------------------- |
| `mapping_launch.py`      | Mapping / SLAM | Wrapper that launches `slam_toolbox` with sync/async mode options and configurable parameters      |
| `localization_launch.py` | Localization   | Wrapper that launches only `nav2_bringup/localization_launch.py` with map file and AMCL parameters |

> **⚠️ Important:** For the `localization_launch.py` file, the app will pass only the map name (e.g., `office.yaml`). Your launch file must construct the full path to the map file, typically using `PathJoinSubstitution` like:
>
> ```python
> 'map': PathJoinSubstitution([pkg_robot_navigation, 'maps', LaunchConfiguration('map')])
> ```
>
> See the example localization launch file below for a complete implementation.

> **💡 Note:** The Mission Planner app can pass custom parameters to these launch files (e.g., map paths, use_sim_time, etc.) via the LaunchWithArgs service. Make sure your launch files accept the parameters you want to configure from the app.

Make sure these files run **stand-alone** before hooking them into the Mission Planner.

</details>

<details>
<summary><b>Example — Mapping Launch File</b></summary>
<br>

Example — start mapping:

````python
# mapping_launch.py - Example template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),
    DeclareLaunchArgument('params',
                          default_value=PathJoinSubstitution(['<robot_navigation_pkg>', 'config', 'slam.yaml']),
                          description='Path to the SLAM Toolbox configuration file')
]


def launch_setup(context, *args, **kwargs):
    # Get parameters
    namespace = LaunchConfiguration('namespace')
    sync = LaunchConfiguration('sync')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    slam_params = LaunchConfiguration('params')

    # Get package paths
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Handle namespace properly for TF remapping
    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    # Get SLAM launch paths
    launch_slam_sync = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_sync_launch.py'])

    launch_slam_async = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    # Create SLAM launch action
    slam = GroupAction([
        PushRosNamespace(namespace),

        # Set remaps for TF and sensor topics
        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),
        SetRemap('/scan', namespace_str + '/scan'),
        SetRemap('/map', namespace_str + '/map'),
        SetRemap('/map_metadata', namespace_str + '/map_metadata'),

        # Include synchronous SLAM if sync=true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_sync),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', slam_params)
            ],
            condition=IfCondition(sync)
        ),

        # Include asynchronous SLAM if sync=false
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_async),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', slam_params)
            ],
            condition=UnlessCondition(sync)
        )
    ])

    return [slam]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

Run it with:

```bash
ros2 launch <robot_pkg> mapping_launch.py use_sim_time:=false
````

</details>

<details>
<summary><b>Example — Localization Launch File</b></summary>
<br>

Example — start localization with a map:

````python
# localization_launch.py - Example template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def generate_launch_description():
    # Replace with your robot's navigation package
    pkg_robot_navigation = get_package_share_directory(
        '<robot_navigation_pkg>')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Point to your localization parameters
    localization_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_robot_navigation, 'config', 'localization.yaml']),
        description='Localization parameters')

    # Map argument
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='map.yaml',
        description='map yaml file to load')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include only the localization launch since nav2 is already running
    localization = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments={'namespace': namespace,
                              'map': PathJoinSubstitution(
                                  [pkg_robot_navigation, 'maps', LaunchConfiguration('map')]),
                              'use_sim_time': use_sim_time,
                              'params_file': LaunchConfiguration('params')}.items()),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization_params_arg)
    ld.add_action(map_arg)
    ld.add_action(localization)
    return ld

Run it with:

```bash
ros2 launch <robot_pkg> localization_launch.py map:=office.yaml
````

</details>

> **⚠️ Warning:** The Mission Planner calls these launch files via its service API; it does **not** ship them.

---

## 🚀 Installation

```bash
# create workspace and setup
mkdir -p ~/nav2_mission_planner_ws/src
cd ~/nav2_mission_planner_ws/src
# clone the repository
git clone https://github.com/botforge-robotics/nav2_mission_planner.git
cd ..
# install dependencies (rosdep is recommended)
rosdep install --from-paths src -i -y
# build
colcon build
source install/setup.bash
```

### Permanent Workspace Setup

To permanently add the workspace to your environment, add the following line to your `~/.bashrc`:

```bash
echo "source ~/nav2_mission_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Mission Planner App

1. Download the Mission Planner app from the [comming soon](https://botforge-robotics.github.io/nav2_mission_planner/)
2. Install the app on your Android device
3. Connect your device to the same network as your robot
4. Open the app and follow the on-screen instructions to connect to your robot

---

## 📄 License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
