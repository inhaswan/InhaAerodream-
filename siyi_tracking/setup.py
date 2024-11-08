from setuptools import setup

package_name = 'object_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seawhan',
    maintainer_email='seawhanl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_tracker = object_tracker.object_tracker:main',
            'position_subscriber = object_tracker.position_reciever:main',
        ],
    },
)
