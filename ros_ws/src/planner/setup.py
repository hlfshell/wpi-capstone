from setuptools import find_packages, setup
import os
from glob import glob

package_name = "planner"

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
        (
            os.path.join("share", package_name, "prompts"),
            glob(os.path.join("prompts", "*")),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob(os.path.join("maps", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Keith Chester",
    maintainer_email="kchester@gmail.com",
    description="LLM powered high level planner",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ai_service = planner.service:main",
        ],
    },
)
