"""Launch helpers shared by the lookie_bringup launch files."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from driver_bringup.models import DEFAULT_MODEL, MODELS, resolve_namespace
from lookie_bringup.config import select_bringup_config


def common_arguments() -> list:
    return [
        DeclareLaunchArgument('model', default_value=DEFAULT_MODEL, choices=list(MODELS),
                              description='Lookie model name.'),
        DeclareLaunchArgument('config', default_value='',
                              description='Absolute bringup YAML path; empty selects config/<model>.yaml.'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace; empty uses the model platform (trio or neo).'),
    ]


def resolve_common(context):
    """Return ``(model, config_path, config, namespace)`` from the common arguments."""
    model = LaunchConfiguration('model').perform(context)
    path, cfg = select_bringup_config(
        model, LaunchConfiguration('config').perform(context), get_package_share_directory)
    ns = resolve_namespace(model, LaunchConfiguration('namespace').perform(context))
    return model, path, cfg, ns


def arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def include(package: str, launch_file: str, arguments: dict) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())
