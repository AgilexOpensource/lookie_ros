"""Lookie drivers: head, torso (lift/fold), dual arms, chassis and joint bridge.

All config files are passed explicitly; they are checked for consistency
(and against the URDF) before any driver node is started.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lookie_driver.layout import (
    ARM_SIDES, CHASSIS_NAMESPACE, HEAD_MODULE, driver_namespace, module_namespaces,
    validate_namespace)
from lookie_driver.models import chassis_package, torso_key, validate

FILE_ARGS = {
    'robot_yaml': 'Motor YAML (head + one torso module, lift or fold).',
    'params_file': 'ROS parameters for head, torso, arm and chassis nodes.',
    'bridge_config': 'Joint bridge YAML (modules + name mapping).',
    'urdf': 'URDF the driver limits and bridged joints are checked against.',
}


def _include(package: str, launch_file: str, arguments: dict) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def _arm_node(namespace: str, params_file: str) -> Node:
    return Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )


def _chassis_node(namespace: str, params_file: str, robot_ns: str) -> Node:
    # The params file names the chassis driver package; its node is <package>_node.
    package = chassis_package(params_file)
    return Node(
        package=package,
        executable=f'{package}_node',
        name=package,
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'odom_frame': f'{robot_ns}/odom',
                                  'base_frame': f'{robot_ns}/base_footprint'}],
    )


def _setup(context, *args, **kwargs):
    ns = validate_namespace(LaunchConfiguration('namespace').perform(context))
    root = driver_namespace(ns)

    files = {}
    for key in FILE_ARGS:
        path = Path(LaunchConfiguration(key).perform(context).strip())
        if not path.is_absolute():
            raise ValueError(f'{key} must be an absolute path, got {str(path)!r}')
        if not path.is_file():
            raise FileNotFoundError(f'{key} not found: {path}')
        files[key] = str(path)

    validate(files['robot_yaml'], files['params_file'], files['bridge_config'], files['urdf'])
    torso = torso_key(files['robot_yaml'])
    sub = module_namespaces(torso)
    common = {'yaml_path': files['robot_yaml'], 'params_file': files['params_file']}

    return [
        _include('head_ros', 'head.launch.py',
                 {'namespace': f'{root}/{sub[HEAD_MODULE]}', **common}),
        _include(f'{torso}_ros', f'{torso}.launch.py',
                 {'namespace': f'{root}/{sub[torso]}', **common}),
        *(_arm_node(f'{root}/{sub[side]}', files['params_file']) for side in ARM_SIDES),
        _chassis_node(f'{root}/{CHASSIS_NAMESPACE}', files['params_file'], ns),
        _include('lookie_driver', 'joint_bridge.launch.py', {
            'bridge_config': files['bridge_config'],
            'namespace': root,
        }),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            description='Robot namespace, e.g. trio. Drivers run under <namespace>/driver.',
        ),
        *(DeclareLaunchArgument(key, description=f'Absolute path. {text}')
          for key, text in FILE_ARGS.items()),
        OpaqueFunction(function=_setup),
    ])
