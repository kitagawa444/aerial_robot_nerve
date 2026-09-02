from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev = LaunchConfiguration("dev")
    baudrate = LaunchConfiguration("baudrate")
    verbosity = LaunchConfiguration("verbosity")
    dds_domain = LaunchConfiguration("spinal_link_dds_domain_id")
    mavlink_bind_address = LaunchConfiguration("mavlink_bind_address")
    mavlink_bind_port = LaunchConfiguration("mavlink_bind_port")
    mavlink_remote_address = LaunchConfiguration("mavlink_remote_address")
    mavlink_remote_port = LaunchConfiguration("mavlink_remote_port")
    use_ros2_bridge = LaunchConfiguration("use_ros2_bridge")
    wifi_interface = LaunchConfiguration("wifi_interface")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="921600"),
            DeclareLaunchArgument("verbosity", default_value="4"),
            DeclareLaunchArgument("spinal_link_dds_domain_id", default_value="0"),
            DeclareLaunchArgument("mavlink_bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("mavlink_bind_port", default_value="14550"),
            DeclareLaunchArgument("mavlink_remote_address", default_value="255.255.255.255"),
            DeclareLaunchArgument("mavlink_remote_port", default_value="14550"),
            DeclareLaunchArgument("use_ros2_bridge", default_value="true"),
            DeclareLaunchArgument("wifi_interface", default_value=""),
            ExecuteProcess(
                cmd=[
                    "MicroXRCEAgent",
                    "serial",
                    "--dev",
                    dev,
                    "-b",
                    baudrate,
                    "-v",
                    verbosity,
                ],
                output="screen",
            ),
            Node(
                package="spinal",
                executable="gcs_gateway",
                name="gcs_gateway",
                output="screen",
                additional_env={
                    "SPINAL_LINK_DDS_DOMAIN_ID": dds_domain,
                    "MAVLINK_BIND_ADDRESS": mavlink_bind_address,
                    "MAVLINK_BIND_PORT": mavlink_bind_port,
                    "MAVLINK_REMOTE_ADDRESS": mavlink_remote_address,
                    "MAVLINK_REMOTE_PORT": mavlink_remote_port,
                },
            ),
            Node(
                package="spinal",
                executable="spinal_ros2_bridge",
                name="spinal_ros2_bridge",
                output="screen",
                condition=IfCondition(use_ros2_bridge),
                parameters=[
                    {
                        "spinal_link_dds_domain_id": dds_domain,
                        "wifi_interface": wifi_interface,
                    }
                ],
            ),
        ]
    )
