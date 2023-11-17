from setuptools import find_packages, setup

package_name = 'interpreter_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bob DeMont',
    maintainer_email='rldemont@wpi.edu',
    description='This interacts with User Node to interpret need',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interpreter = interpreter_package.interpreter:main'
        ],
    },
)
