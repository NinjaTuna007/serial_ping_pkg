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
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ('share/' + package_name + '/data', glob('data/*') if os.path.exists('data') else []),
        ('share/' + package_name + '/local', [f for f in glob('local/*') if os.path.isfile(f)]),
        ('share/' + package_name + '/local/scripts', glob('local/scripts/*') if os.path.exists('local/scripts') else []),
        ('share/' + package_name + '/local/USB_pinout', glob('local/USB pinout/*') if os.path.exists('local/USB pinout') else []),
        ('share/' + package_name + '/NM3Firmware', glob('NM3Firmware/*') if os.path.exists('NM3Firmware') else []),
        ('share/' + package_name + '/User_Manual', glob('User Manual/*') if os.path.exists('User Manual') else []),
    ],
    install_requires=['setuptools', 'pyyaml', 'pyserial', 'pandas'],
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='A comprehensive ROS 2 package for underwater acoustic communication and ranging using Succorfish Delphis modems',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_ping_node = serial_ping_pkg.serial_ping_node:main',
            'smarc_pos_broadcast_node = serial_ping_pkg.smarc_pos_broadcast_node:main',
            'smarc_pos_receiver_node = serial_ping_pkg.smarc_pos_receiver_node:main',
            'SerialBroadcastReceiver = serial_ping_pkg.SerialBroadcastReceiver:main',
            'informed_leader_node = serial_ping_pkg.informed_leader_follower.informed_leader_node:main',
            'informed_follower_node = serial_ping_pkg.informed_leader_follower.informed_follower_node:main',
        ],
    },
)