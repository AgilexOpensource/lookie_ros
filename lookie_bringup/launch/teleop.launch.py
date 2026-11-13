"""Lookie VR teleop: xr_media_ros (XR card only) + lookie_teleop, using this model's configs.

kstone_ros is not started here; launch it separately before teleoperating.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from lookie_bringup.config import load_teleop_config
from lookie_bringup.launch_utils import arg, common_arguments, include, resolve_common


def _setup(context, *args, **kwargs):
    config_path, _, ns = resolve_common(context)
    teleop = load_teleop_config(config_path, get_package_share_directory)

    actions = []
    if arg(context, 'xr') == 'true':
        actions.append(include('xr_media_ros', 'xr_media.launch.py', {'config': str(teleop.xr_media)}))
    if arg(context, 'teleop') == 'true':
        actions.append(include('lookie_teleop', 'teleop.launch.py', {
            'params_file': ','.join(str(p) for p in teleop.lookie_teleop),
            'namespace': ns,
        }))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *common_arguments(),
        DeclareLaunchArgument('xr', default_value='true', choices=['true', 'false'],
                              description='Start xr_media_ros; false attaches to a running XR node.'),
        DeclareLaunchArgument('teleop', default_value='true', choices=['true', 'false'],
                              description='Start lookie_teleop.'),
        OpaqueFunction(function=_setup),
    ])
