"""lookie_control kstone_ros, using this model's bringup config."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger

from lookie_bringup.config import feedback_timeout_warnings, load_bridge_topics, load_kstone_config
from lookie_bringup.launch_utils import arg, common_arguments, include, resolve_common
from lookie_driver.layout import CHASSIS_NAMESPACE, DRIVER_NAMESPACE


def _setup(context, *args, **kwargs):
    config_path, cfg, ns = resolve_common(context)
    topics = load_bridge_topics(cfg.bridge_config)
    kstone_config = load_kstone_config(config_path, get_package_share_directory)
    for warning in feedback_timeout_warnings(cfg.bridge_config, kstone_config):
        get_logger('lookie_bringup').warning(warning)
    driver = DRIVER_NAMESPACE
    return [include('lookie_control', 'kstone.launch.py', {
        'namespace': ns,
        'config_file': str(kstone_config),
        'urdf': str(cfg.collision_urdf),
        'feedback_topic': f'{driver}/{topics.feedback_topic}',
        'control_topic': f'{driver}/{topics.control_topic}',
        'base_command_topic': f'{driver}/{CHASSIS_NAMESPACE}/cmd_vel',
        'odom_topic': f'{driver}/{CHASSIS_NAMESPACE}/odom',
        'realtime': arg(context, 'realtime'),
        'debug_port': arg(context, 'debug_port'),
        'use_sim_time': arg(context, 'use_sim_time'),
    })]


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
