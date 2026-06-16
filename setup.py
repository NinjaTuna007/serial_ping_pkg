from setuptools import setup
import os
from glob import glob

package_name = 'serial_ping_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + '.informed_leader_follower',
        package_name + '.acoustic_relay',
        package_name + '.tuper_owtt',
        package_name + '.owtt_beacon',
        package_name + '.ping_estimator',
    ],
    data_files=[
    	('share/ament_index/resource_index/packages',
        ['resource/serial_ping_pkg']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ('share/' + package_name + '/data', glob('data/*') if os.path.exists('data') else []),
        ('share/' + package_name + '/local', [f for f in glob('local/*') if os.path.isfile(f)]),
        ('share/' + package_name + '/local/scripts', [f for f in glob('local/scripts/*') if os.path.isfile(f)]),
        ('share/' + package_name + '/local/USB_pinout', glob('local/USB pinout/*') if os.path.exists('local/USB pinout') else []),
        ('share/' + package_name + '/NM3Firmware', glob('NM3Firmware/*') if os.path.exists('NM3Firmware') else []),
        ('share/' + package_name + '/User_Manual', glob('User Manual/*') if os.path.exists('User Manual') else []),
    ],
    install_requires=['setuptools', 'pyyaml', 'pyserial', 'pandas', 'numpy', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='A comprehensive ROS 2 package for underwater acoustic communication and ranging using Succorfish Delphis modems',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_ping_node = serial_ping_pkg.serial_ping_node:main',
            'smarc_pos_broadcast_node = serial_ping_pkg.acoustic_relay.smarc_pos_broadcast_node:main',
            'smarc_pos_receiver_node = serial_ping_pkg.acoustic_relay.smarc_pos_receiver_node:main',
            'SerialBroadcastReceiver = serial_ping_pkg.SerialBroadcastReceiver:main',
            'informed_leader_node = serial_ping_pkg.informed_leader_follower.informed_leader_node:main',
            'informed_follower_node = serial_ping_pkg.informed_leader_follower.informed_follower_node:main',
            'single_target_ping_node = serial_ping_pkg.single_target_ping_node:main',
            'owtt_follower_node = serial_ping_pkg.tuper_owtt.owtt_follower_node:main',
            'owtt_leader_node = serial_ping_pkg.tuper_owtt.owtt_leader_node:main',
            'teensy_cmd = serial_ping_pkg.tuper_owtt.teensy_cmd:main',
            'owtt_beacon_node = serial_ping_pkg.owtt_beacon.beacon_node:main',
            'owtt_surface_unit_node = serial_ping_pkg.owtt_beacon.surface_unit_node:main',
            'owtt_inference_node = serial_ping_pkg.owtt_beacon.inference_node:main',
            'owtt_fake_mqtt = serial_ping_pkg.owtt_beacon.fake_mqtt_publisher:main',
            'modem_ping_estimator_node = serial_ping_pkg.ping_estimator.modem_ping_estimator_node:main',
            'modem_ping_cmd = serial_ping_pkg.ping_estimator.modem_ping_cmd:main',
        ],
    },
)
