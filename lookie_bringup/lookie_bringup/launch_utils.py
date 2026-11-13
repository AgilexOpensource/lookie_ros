"""Launch helpers shared by the lookie_bringup launch files."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from lookie_bringup.config import MODEL_ENV, available_models, select_bringup_config
from lookie_driver.layout import validate_namespace


def common_arguments() -> list:
    models = available_models(Path(get_package_share_directory('lookie_bringup')) / 'config')
    return [
        DeclareLaunchArgument('model', default_value=EnvironmentVariable(MODEL_ENV, default_value=''),
                              description=f'Lookie model, a directory under lookie_bringup config/; '
                                          f'defaults to ${MODEL_ENV}. Valid choices are: {models}'),
        DeclareLaunchArgument('config', default_value='',
                              description='Absolute bringup YAML path; empty selects '
                                          'config/<model>/bringup.yaml.'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace; empty uses the bringup YAML namespace.'),
    ]


def resolve_common(context):
    """Return ``(config_path, config, namespace)`` from the common arguments."""
    path, cfg = select_bringup_config(
        arg(context, 'model').strip(), arg(context, 'config'), get_package_share_directory)
    ns = arg(context, 'namespace').strip()
    return path, cfg, validate_namespace(ns) if ns else cfg.namespace


def arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def include(package: str, launch_file: str, arguments: dict) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())
