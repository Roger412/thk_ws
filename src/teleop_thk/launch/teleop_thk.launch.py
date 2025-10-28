from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Argument to control whether rqt_reconfigure launches
    reconfigure_arg = DeclareLaunchArgument(
        "reconfigure",
        default_value="false",
        description="Set to true to launch rqt_reconfigure GUI"
    )

    # --- teleop node ---
    teleop_node = Node(
        package="teleop_thk",
        executable="teleop_thk",
        name="teleop_thk",
        output="screen"
    )

    # --- static rf (map -> odom) ---
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    # --- RViz2  ---
    rviz_config_path = os.path.join(
        get_package_share_directory("teleop_thk"),
        "rviz",
        "rviz_config.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    # --- rqt_reconfigure ---
    rqt_reconfigure = ExecuteProcess(
        cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
        condition=IfCondition(LaunchConfiguration("reconfigure")),
        output="screen"
    )

    return LaunchDescription([
        reconfigure_arg,
        teleop_node,
        static_tf,
        rviz,
        rqt_reconfigure
    ])
