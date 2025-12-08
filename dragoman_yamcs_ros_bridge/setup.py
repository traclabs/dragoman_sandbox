from setuptools import setup

package_name = 'dragoman_yamcs_ros_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TRACLabs Robotics',
    maintainer_email='robotics@traclabs.com',
    description='YAMCS to ROS2 bridge package for bidirectional telemetry and command handling',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yamcs_ros_bridge = dragoman_yamcs_ros_bridge.yamcs_ros_bridge:main',
            'curiosity_yamcs_ros_bridge = dragoman_yamcs_ros_bridge.curiosity_yamcs_ros_bridge:main',
        ],
    },
)
