"""Lookie drivers + robot_state_publisher + RViz, with optional joint slider control."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from driver_bringup.models import validate
from lookie_bringup.config import load_bridge_topics
from lookie_bringup.launch_utils import arg, common_arguments, include, resolve_common


def _setup(context, *args, **kwargs):
    model, _, cfg, ns = resolve_common(context)
    urdf = Path(get_package_share_directory('lookie_description')) / 'urdf' / f'{model}.urdf'
    validate(model, cfg.robot_yaml, cfg.params_file, cfg.bridge_config, urdf_path=urdf)
    topics = load_bridge_topics(cfg.bridge_config)

    actions = []
    if arg(context, 'driver') == 'true':
        actions.append(include('driver_bringup', 'lookie.launch.py', {
            'model': model,
            'robot_yaml': str(cfg.robot_yaml),
            'params_file': str(cfg.params_file),
            'bridge_config': str(cfg.bridge_config),
            'namespace': ns,
        }))
    actions.append(include('lookie_description', 'display.launch.py', {
        'model': model,
        'namespace': ns,
        'follow': 'true',
        'control': arg(context, 'control'),
        'gui': arg(context, 'gui'),
        'rviz': arg(context, 'rviz'),
        'fixed_frame': arg(context, 'fixed_frame'),
        'feedback_topic': f'driver/{topics.feedback_topic}',
        'control_topic': f'driver/{topics.control_topic}',
        'initial': str(cfg.initial),
    }))
    return actions


def _flag(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, choices=['true', 'false'],
                                 description=description)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *common_arguments(),
        _flag('driver', 'true', 'Start driver_bringup; false attaches to running drivers.'),
        _flag('control', 'false', 'Start joint sliders; the robot moves to the initial pose at once.'),
        _flag('gui', 'true', 'Use joint_state_publisher_gui sliders when control:=true.'),
        _flag('rviz', 'true', 'Start RViz; robot_state_publisher runs either way.'),
        DeclareLaunchArgument('fixed_frame', default_value='odom',
                              description='RViz Fixed Frame without the namespace; use base_footprint '
                                          'when no chassis odometry is running.'),
        OpaqueFunction(function=_setup),
    ])
