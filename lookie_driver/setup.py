import os
from glob import glob

from setuptools import setup

package_name = "lookie_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="Lookie drivers: launch files, config checks, and joint name bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_bridge_node = lookie_driver.joint_bridge_node:main",
        ],
    },
)
