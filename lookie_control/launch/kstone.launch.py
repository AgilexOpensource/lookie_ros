"""kstone_ros with an explicit engine config, URDF, namespace and topics."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _file(context, name: str) -> Path:
    path = Path(_arg(context, name))
    if not path.is_absolute():
        raise ValueError(f'{name} must be an absolute path, got {str(path)!r}')
    if not path.is_file():
        raise FileNotFoundError(f'{name} not found: {path}')
    return path


def _setup(context, *args, **kwargs):
    ns = _arg(context, 'namespace').strip('/')
    if not ns:
        raise ValueError('namespace must not be empty')
    urdf = _file(context, 'urdf')
    # <share>/<pkg>/urdf/<name>.urdf -> <share>, so package:// mesh URIs resolve.
    package_dirs = _arg(context, 'package_dirs') or str(urdf.parents[2])

    params = {
        'config_file': str(_file(context, 'config_file')),
        'config_variables': json.dumps({'urdf': str(urdf), 'package_dirs': package_dirs}),
        'frame_prefix': f'{ns}/',
        'world_frame': 'odom',
        'base_frame': 'base_footprint',
        'feedback_joint_topic': _arg(context, 'feedback_topic').strip('/'),
        'joint_command_topic': _arg(context, 'control_topic').strip('/'),
        'base_command_topic': _arg(context, 'base_command_topic').strip('/'),
        'use_sim_time': _arg(context, 'use_sim_time') == 'true',
    }
    realtime = _arg(context, 'realtime')
    if realtime != 'inherit':
        params['realtime'] = realtime == 'true'
    debug_port = _arg(context, 'debug_port')
    if debug_port != 'inherit':
        params['debug_port'] = int(debug_port)
    bridge = Path(get_package_share_directory('kstone_ros')) / 'config' / 'bridge.yaml'
    return [Node(package='kstone_ros', executable='kstone_node', name='kstone', namespace=ns,
                 output='screen', parameters=[str(bridge), params],
                 remappings=[('~/odom', _arg(context, 'odom_topic').strip('/'))])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Robot namespace, e.g. trio.'),
        DeclareLaunchArgument('config_file', description='Absolute kstone engine config path.'),
        DeclareLaunchArgument('urdf', description='Absolute URDF path, substituted for ${urdf}.'),
        DeclareLaunchArgument('package_dirs', default_value='',
                              description='Root for package:// URIs in the URDF; empty uses '
                                          'three levels above the URDF (the install share dir).'),
        DeclareLaunchArgument('feedback_topic', default_value='driver/feedback/joint_states',
                              description='Joint feedback topic relative to <namespace>.'),
        DeclareLaunchArgument('control_topic', default_value='driver/control/joint_states',
                              description='Joint command topic relative to <namespace>.'),
        DeclareLaunchArgument('base_command_topic', default_value='driver/chassis/cmd_vel',
                              description='Chassis velocity command topic relative to <namespace>.'),
        DeclareLaunchArgument('odom_topic', default_value='driver/chassis/odom',
                              description='Chassis odometry topic relative to <namespace>.'),
        DeclareLaunchArgument('realtime', default_value='inherit', choices=['inherit', 'true', 'false'],
                              description='inherit keeps the kstone config thread settings.'),
        DeclareLaunchArgument('debug_port', default_value='inherit',
                              description='kstone DebugServer port; 0 = off.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_setup),
    ])
