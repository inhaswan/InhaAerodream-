from setuptools import setup, find_packages
import os
from glob import glob


package_name = 'object_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_air',
    maintainer_email='alpha12334@naver.com',
    description='Take Point Cloud Data from Gazebo, and Try to Do Some Pre-Processing',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "offboard_control = object_detection.offboard_control:main",
            "point_cloud = object_detection.point_cloud:main",
            "test = object_detection.test:main",
        ],
    },
)