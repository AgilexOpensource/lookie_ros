import os
from glob import glob

from setuptools import setup

package_name = "lookie_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="Lookie kstone_ros launch file",
    license="Apache-2.0",
)
