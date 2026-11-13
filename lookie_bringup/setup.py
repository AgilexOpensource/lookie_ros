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
        (os.path.join("lib", package_name), ["scripts/can_bind.sh"]),
        *[(os.path.join("share", package_name, d.rstrip("/")), glob(os.path.join(d, "*.yaml")))
          for d in sorted(glob("config/*/"))],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="Lookie bringup: drivers, kstone and teleop launch files",
    license="Apache-2.0",
)
