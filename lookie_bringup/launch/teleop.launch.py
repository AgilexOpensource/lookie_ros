"""Lookie VR teleop: xr_media_ros (XR card only) + lookie_teleop, using this model's configs.

kstone_ros is not started here; launch it separately before teleoperating. twist_mux arbitrates the
chassis velocity input of kstone (config/twist_mux.yaml lists the sources), so lookie_teleop publishes on
lookie_teleop/cmd_vel instead of writing to the kstone chassis port directly.
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch_ros.actions import Node

from lookie_bringup.config import load_teleop_config
from lookie_bringup.launch_utils import arg, common_arguments, include, resolve_common

# Source topic of lookie_teleop and the kstone chassis velocity port it reaches through twist_mux; both
# are relative to the robot namespace.
TELEOP_CMD_VEL = 'lookie_teleop/cmd_vel'
KSTONE_CHASSIS_CMD_VEL = 'kstone/chassis/control/cmd_vel'


def _twist_mux(ns: str):
    """The arbitration node, or None when twist_mux is not installed: then teleop writes its topic directly
    and nothing else may publish on it."""
    try:
        share = Path(get_package_share_directory('lookie_bringup'))
        get_package_share_directory('twist_mux')
    except PackageNotFoundError:
        get_logger('lookie_bringup').warning(
            'twist_mux is not installed: lookie_teleop drives the chassis directly, so no other velocity '
            'source may publish on the kstone chassis input (sudo apt install ros-humble-twist-mux)')
        return None
    return Node(package='twist_mux', executable='twist_mux', name='twist_mux', namespace=ns, output='screen',
                parameters=[str(share / 'config' / 'twist_mux.yaml')],
                remappings=[('cmd_vel_out', KSTONE_CHASSIS_CMD_VEL)])


def _setup(context, *args, **kwargs):
    config_path, _, ns = resolve_common(context)
    teleop = load_teleop_config(config_path, get_package_share_directory)

    actions = []
    if arg(context, 'xr') == 'true':
        actions.append(include('xr_media_ros', 'xr_media.launch.py', {'config': str(teleop.xr_media)}))
    mux = None
    if arg(context, 'teleop') == 'true':
        mux = _twist_mux(ns) if arg(context, 'mux') == 'true' else None
        params = {
            'params_file': ','.join(str(p) for p in teleop.lookie_teleop),
            'namespace': ns,
        }
        if mux is not None:
            params['chassis_cmd_topic'] = TELEOP_CMD_VEL
        actions.append(include('lookie_teleop', 'teleop.launch.py', params))
    if mux is not None:
        actions.insert(0, mux)
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *common_arguments(),
        DeclareLaunchArgument('xr', default_value='true', choices=['true', 'false'],
                              description='Start xr_media_ros; false attaches to a running XR node.'),
        DeclareLaunchArgument('teleop', default_value='true', choices=['true', 'false'],
                              description='Start lookie_teleop.'),
        DeclareLaunchArgument('mux', default_value='true', choices=['true', 'false'],
                              description='Start twist_mux in front of the kstone chassis velocity input; '
                                          'false makes lookie_teleop its only writer.'),
        OpaqueFunction(function=_setup),
    ])
