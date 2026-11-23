"""Launch only the Lookie whole-body teleop node.

xr_media_ros and kstone_ros must be started separately. Robot-specific parameter files
live in the bringup package (e.g. lookie_bringup/config/lookie_teleop/); pass them via params_file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context, *args, **kwargs):
    value = LaunchConfiguration("params_file").perform(context)
    parameters = [f.strip() for f in value.split(",") if f.strip()]
    topic = LaunchConfiguration("chassis_cmd_topic").perform(context).strip()
    if topic:
        parameters.append({"chassis_cmd_topic": topic})
    return [Node(
        package="lookie_teleop",
        executable="lookie_teleop_node",
        name="lookie_teleop",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=parameters,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value="",
            description="Comma-separated parameter YAMLs applied in order; empty uses node defaults"),
        DeclareLaunchArgument("namespace", default_value="", description="Node namespace"),
        DeclareLaunchArgument(
            "chassis_cmd_topic", default_value="",
            description="Chassis velocity topic relative to the namespace; empty uses the kstone chassis "
                        "velocity port (set it when twist_mux arbitrates that port)"),
        OpaqueFunction(function=_node),
    ])
