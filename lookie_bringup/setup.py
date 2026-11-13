import os
from glob import glob

from setuptools import setup

package_name = "lookie_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "kstone_ros"), glob("config/kstone_ros/*.yaml")),
        (os.path.join("share", package_name, "config", "lookie_wbc_teleop"), glob("config/lookie_wbc_teleop/*.yaml")),
        (os.path.join("share", package_name, "config", "xr_media_ros"), glob("config/xr_media_ros/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="Lookie bringup: drivers combined with RViz joint control",
    license="Apache-2.0",
)
