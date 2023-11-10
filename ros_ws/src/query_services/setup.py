from setuptools import find_packages, setup

package_name = 'query_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[query_services]
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brigitte Broszus',
    maintainer_email='bbroszus@wpi.edu',
    description='State query service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = query_services.description_query_service:main',
            'client = query_services.description_query_client:main',

        ],
    },
)
