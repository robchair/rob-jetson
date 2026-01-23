from setuptools import find_packages, setup

package_name = 'robovoice_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rob',
    maintainer_email='jedelist@bu.edu',
    description='ROS2 wrapper for roboVoice that publishes cmd_vel.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_cmd_node = robovoice_ros2.voice_cmd_node:main',
        ],
    },
)
