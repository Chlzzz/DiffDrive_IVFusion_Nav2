import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("hik_camera"), "config", "default.rviz"]
    # )
    
    config_dir = os.path.join(get_package_share_directory('ipcamera'), 'config')
    
    param_config = os.path.join(config_dir, "ipcamera.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["ipcamera"]["ros__parameters"]

    config_file = 'file://' + os.path.join(config_dir, "camera_info.yaml")

    
    hik_camera_node = Node(
        package="ipcamera",
        executable="ipcamera",
        name="ipcamera",
        parameters=[params,
                    {"camera_calibration_file": config_file}],
        output="screen"
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        #arguments=["-d", rviz_config_file],
        #parameters=[{"use_sim_time": True}],
        condition=IfCondition(gui),
    )

     
    nodes = [
        hik_camera_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
