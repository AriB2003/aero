from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package resource index and package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zaraius',
    maintainer_email='zaraiusbillimoria@gmail.com',
    description='ROS2 Aero Project Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'talker = my_package.my_node:main',
            'listener = my_package.subscriber_test_function:main',
            'telemetry = my_package.telemetry:main',
        ],
    },
)
