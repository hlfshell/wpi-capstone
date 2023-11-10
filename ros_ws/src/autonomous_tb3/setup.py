import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomous_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, 'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, 'config'), glob("config/*")),
        (os.path.join("share", package_name, 'worlds/simple_maze'), glob("worlds/simple_maze/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vagrant',
    maintainer_email='vagrant@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_pub = autonomous_tb3.occupancy_grid_pub:main',
            'sdf_spawner = autonomous_tb3.spawn_entity:main'
        ],
    },
)
