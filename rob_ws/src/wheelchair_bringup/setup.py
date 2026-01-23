from setuptools import setup
import os
from glob import glob

package_name = 'wheelchair_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rob',
    maintainer_email='jedelist@bu.edu',
    description='Bringup launch/config for wheelchair MVP',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_gate_node = wheelchair_bringup.safety_gate_node:main',
        ],
    },
)

