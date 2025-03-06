from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

TURTLEBOT3_SIM_SCAN_SIZE = 360


def generate_launch_description():
    ld = LaunchDescription(
        [
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle"),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value="/opt/ros/humble/share/turtlebot3_gazebo/models",
            ),
        ]
    )
    demo_bringup_dir = get_package_share_directory("demo_bringup")
    nav2_launch_file = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch/unique_multi_tb3_simulation_launch.py",
    )
    robot1_params_path = PathJoinSubstitution([demo_bringup_dir, "config", "robot1_tb3_nav2.yaml"])
    robot2_params_path = PathJoinSubstitution([demo_bringup_dir, "config", "robot2_tb3_nav2.yaml"])
    # params_path = PathJoinSubstitution([demo_bringup_dir, "config", "tb3_nav2.yaml"])
    rviz_path = PathJoinSubstitution(
        [demo_bringup_dir, "launch", "other", "nav2_default_view.rviz"]
    )

    

    nav2_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments=[
            ("headless", "False"),
            # ("params_file", params_path),
            ("robot1_params_file", robot1_params_path),
            ("robot2_params_file", robot2_params_path),
            ("rviz_config", rviz_path),
            ("use_rviz", "True"),
        ],
    )
    
    ld.add_action(nav2_ld)
    robots = [
        "robot1",
        "robot2",
    ]
    
    

    scan_node = Node(
        package="scan_modifier",
        executable="scan_node",
        # namespace = robot,
        parameters=[{"scan_ranges_size": TURTLEBOT3_SIM_SCAN_SIZE}],
    )

    spin_config_node = Node(
        package="topic_param_bridge",
        executable="param_bridge",
        # namespace=robot,
    )
        
    ld.add_action(scan_node)
    ld.add_action(spin_config_node)
    
    
    return ld