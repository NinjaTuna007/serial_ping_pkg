from setuptools import setup
import os
from glob import glob

package_name = 'serial_ping_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    	('share/ament_index/resource_index/packages',
        ['resource/serial_ping_pkg']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/serial_config.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='abhinav',
    maintainer_email='your_email@example.com',
    description='A ROS 2 node to ping over serial and calculate distance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_ping_node = serial_ping_pkg.serial_ping_node:main',
        ],
    },
)
# This setup script is for a ROS 2 package named 'serial_ping_pkg'.
