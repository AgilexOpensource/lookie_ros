"""kstone_ros for a Lookie model, wired to the drivers under /<namespace>/driver."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from driver_bringup.models import CHASSIS_NAMESPACE
from lookie_bringup.config import load_bridge_topics, load_kstone_config
from lookie_bringup.launch_utils import arg, common_arguments, resolve_common


def _setup(context, *args, **kwargs):
    model, config_path, cfg, ns = resolve_common(context)
    topics = load_bridge_topics(cfg.bridge_config)
    desc = Path(get_package_share_directory('lookie_description'))

    params = {
        'config_file': str(load_kstone_config(config_path, get_package_share_directory)),
        'config_variables': json.dumps({
            'urdf': str(desc / 'urdf' / f'{model}_collision.urdf'),
            'package_dirs': str(desc.parent),
        }),
        'frame_prefix': f'{ns}/',
        'world_frame': 'odom',
        'base_frame': 'base_footprint',
        'feedback_joint_topic': f'driver/{topics.feedback_topic}',
        'joint_command_topic': f'driver/{topics.control_topic}',
        'base_command_topic': f'driver/{CHASSIS_NAMESPACE}/cmd_vel',
        'use_sim_time': arg(context, 'use_sim_time') == 'true',
    }
    realtime = arg(context, 'realtime')
    if realtime != 'inherit':
        params['realtime'] = realtime == 'true'
    debug_port = arg(context, 'debug_port')
    if debug_port != 'inherit':
        params['debug_port'] = int(debug_port)
    bridge = Path(get_package_share_directory('kstone_ros')) / 'config' / 'bridge.yaml'
    return [Node(package='kstone_ros', executable='kstone_node', name='kstone', namespace=ns,
                 output='screen', parameters=[str(bridge), params],
                 remappings=[('~/odom', f'driver/{CHASSIS_NAMESPACE}/odom')])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *common_arguments(),
        DeclareLaunchArgument('realtime', default_value='inherit', choices=['inherit', 'true', 'false'],
                              description='inherit keeps the kstone config thread settings.'),
        DeclareLaunchArgument('debug_port', default_value='inherit',
                              description='kstone DebugServer port; 0 = off.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_setup),
    ])
