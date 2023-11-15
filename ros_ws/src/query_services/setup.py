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
            'description_service = query_services.description_query_service:main',
            'description_client = query_services.description_query_client:main',

            'id_service = query_services.id_query_service:main',
            'id_client = query_services.id_query_client:main',
            
            'add_object_service = query_services.new_object_service:main',
            'add_object_client = query_services.new_object_client:main',

        ],
    },
)
