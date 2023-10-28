import os
from glob import glob

from setuptools import find_packages, setup

package_name = "llm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Keith Chester",
    maintainer_email="kchester@gmail.com",
    description="LLM provider wrapper service",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "llm_service = llm.service:main",
        ],
    },
)
