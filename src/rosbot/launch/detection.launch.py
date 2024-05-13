import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rosbot_params_file = LaunchConfiguration("rosbot_params_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )

    rosbot_params_file_arg = DeclareLaunchArgument(
        "rosbot_params_file",
        default_value=os.path.join(
            get_package_share_directory("rosbot"), "config", "detection.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for the detection node",
    )

    detection_node = Node(
        package="rosbot",
        executable="detection",
        name="detection",
        output="screen",
        remappings=[
            ("/image", "/camera/color/image_raw")
        ],
        parameters=[rosbot_params_file, {"use_sim_time": use_sim_time}],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("rosbot"), "rviz", "detection.rviz"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(rosbot_params_file_arg)
    ld.add_action(detection_node)
    ld.add_action(rviz2_node)

    return ld