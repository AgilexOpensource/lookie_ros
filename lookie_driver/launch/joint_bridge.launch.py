from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context, *args, **kwargs):
    config = LaunchConfiguration('bridge_config').perform(context)
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    if not ns:
        ns = str((yaml.safe_load(Path(config).read_text(encoding='utf-8')) or {}).get('namespace', '')).strip('/')
    return [Node(
        package='lookie_driver',
        executable='joint_bridge_node',
        name='joint_bridge_node',
        namespace=ns,
        output='screen',
        parameters=[{'config_file': config}],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_config',
            description='Absolute joint bridge YAML path (modules + name mapping).',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Driver root namespace (e.g. trio/driver); empty uses the YAML value.',
        ),
        OpaqueFunction(function=_node),
    ])
