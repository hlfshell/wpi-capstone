import os
from glob import glob

from setuptools import find_packages, setup

package_name = "explorer"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Keith Chester",
    maintainer_email="kchester@gmail.com",
    description="Upon receiving an occupancy map, identify where the robot needs to reasonably explore next",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explorer = explorer.service:main",
        ],
    },
)
