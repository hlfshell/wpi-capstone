from setuptools import find_packages, setup
import os
from glob import glob

package_name = "litterbug"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*"))),
        (
            os.path.join("share", package_name, "items"),
            glob(os.path.join("items", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Keith Chester",
    maintainer_email="kchester@gmail.com",
    description="Litterbug handles item interaction and simulates vision",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "litterbug_service = litterbug.litterbug:main",
        ],
    },
)
