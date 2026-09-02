from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dev = LaunchConfiguration("dev")
    baudrate = LaunchConfiguration("baudrate")
    verbosity = LaunchConfiguration("verbosity")

    return LaunchDescription([
        DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="921600"),
        DeclareLaunchArgument("verbosity", default_value="6"),

        ExecuteProcess(
            cmd=[
                "MicroXRCEAgent",
                "serial",
                "--dev", dev,
                "-b", baudrate,
                "-v", verbosity,
            ],
            output="screen",
        ),
    ])
