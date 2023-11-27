import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'interpreter_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('*.prompt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bob DeMont',
    maintainer_email='rldemont@wpi.edu',
    description='This package uses LLM to get users desired object.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interpreter = interpreter_pkg.interpreter:main'
        ],
    },
)
