from setuptools import find_packages, setup

package_name = 'wheelchair_localization'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmdvel_odom = wheelchair_localization.cmdvel_odom:main',
            'encoder_serial_node = wheelchair_localization.encoder_serial_node:main',
<<<<<<< HEAD
            'encoder_odom = wheelchair_localization.encoder_odom:main',
            'imu_node= wheelchair_localization.imu_node:main'
=======
            'encoder_odom = wheelchair_localization.encoder_odom:main'
>>>>>>> bryan_camera
        ],
    },
)
