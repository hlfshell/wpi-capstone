from setuptools import find_packages, setup
import os
from glob import glob

package_name = "litterbug"

data_files = [
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
]

models_share_path = os.path.join("share", package_name)
for path, _, files in os.walk("models"):
    # Drop models from path
    list_entry = (
        models_share_path + "/" + path,
        # Exclude models from path
        [
            os.path.join(path, f)  # [len("models") + 1 :]
            for f in files
            if not f.startswith(".")
        ],
    )
    data_files.append(list_entry)

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
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
