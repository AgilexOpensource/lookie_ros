"""Lookie drivers + robot_state_publisher, optional RViz; sim:=true replaces the robot with joint sliders."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from lookie_driver.layout import DRIVER_NAMESPACE
from lookie_driver.models import validate
from lookie_bringup.config import load_bridge_topics
from lookie_bringup.launch_utils import arg, common_arguments, include, resolve_common


def _setup(context, *args, **kwargs):
    _, cfg, ns = resolve_common(context)
    validate(cfg.robot_yaml, cfg.params_file, cfg.bridge_config, urdf_path=cfg.urdf)
    topics = load_bridge_topics(cfg.bridge_config)

    sim = arg(context, 'sim') == 'true'
    feedback_topic = f'{DRIVER_NAMESPACE}/{topics.feedback_topic}'
    actions = []
    if not sim and arg(context, 'driver') == 'true':
        actions.append(include('lookie_driver', 'drivers.launch.py', {
            'robot_yaml': str(cfg.robot_yaml),
            'params_file': str(cfg.params_file),
            'bridge_config': str(cfg.bridge_config),
            'urdf': str(cfg.urdf),
            'namespace': ns,
        }))
    # display.launch.py selects lookie_description urdf/<model>.urdf by stem.
    actions.append(include('lookie_description', 'display.launch.py', {
        'model': cfg.urdf.stem,
        'namespace': ns,
        'follow': 'true',
        # Sliders only ever publish feedback, so they cannot move a real robot.
        'control': 'true' if sim else 'false',
        'gui': arg(context, 'gui'),
        'rviz': arg(context, 'rviz'),
        'fixed_frame': arg(context, 'fixed_frame'),
        'feedback_topic': feedback_topic,
        'control_topic': feedback_topic,
        'initial': str(cfg.initial),
    }))
    return actions


def _flag(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, choices=['true', 'false'],
                                 description=description)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *common_arguments(),
        _flag('sim', 'false', 'No robot: skip drivers, joint sliders publish the feedback topic.'),
        _flag('driver', 'true', 'Start lookie_driver; false attaches to running drivers.'),
        _flag('gui', 'true', 'With sim:=true, use joint_state_publisher_gui sliders; false holds the initial pose.'),
        _flag('rviz', 'false', 'Start RViz; robot_state_publisher runs either way.'),
        DeclareLaunchArgument('fixed_frame', default_value='odom',
                              description='RViz Fixed Frame without the namespace; use base_footprint '
                                          'when no chassis odometry is running.'),
        OpaqueFunction(function=_setup),
    ])
