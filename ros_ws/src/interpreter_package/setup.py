import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'interpreter_package'

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
    maintainer='vagrant',
    maintainer_email='rldemont@wpi.edu',
    description='Node to interpret user request',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interpreter = interpreter_package.interpreter:main'
        ],
    },
)
