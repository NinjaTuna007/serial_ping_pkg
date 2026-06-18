from setuptools import setup
import os
from glob import glob

package_name = 'serial_ping_pkg'


def collect_data_files(src_dir, dest_prefix):
    """Install every file under src_dir, preserving its sub-directory layout.

    Lets configs/launches be grouped per sub-package (e.g. config/ping_estimator_action/)
    while still landing under share/<pkg>/<dest_prefix>/<subdir>/.
    """
    entries = []
    for root, _, files in os.walk(src_dir):
        files = [f for f in files if not f.endswith('.pyc')]
        if not files:
            continue
        rel = os.path.relpath(root, src_dir)
        dest = dest_prefix if rel == '.' else os.path.join(dest_prefix, rel)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + '.common',
        package_name + '.tuper_twtt',
        package_name + '.acoustic_relay',
        package_name + '.tuper_owtt',
        package_name + '.owtt_beacon',
        package_name + '.ping_estimator_action',
    ],
    data_files=[
    	('share/ament_index/resource_index/packages',
        ['resource/serial_ping_pkg']),
        ('share/' + package_name, ['package.xml']),
        *collect_data_files('config', 'share/' + package_name + '/config'),
        *collect_data_files('launch', 'share/' + package_name + '/launch'),
        ('share/' + package_name + '/data', glob('data/*') if os.path.exists('data') else []),
    ],
    install_requires=['setuptools', 'pyyaml', 'pyserial', 'pandas', 'numpy', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='A comprehensive ROS 2 package for underwater acoustic communication and ranging using Succorfish Delphis modems',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_ping_node = serial_ping_pkg.common.serial_ping_node:main',
            'smarc_pos_broadcast_node = serial_ping_pkg.acoustic_relay.smarc_pos_broadcast_node:main',
            'smarc_pos_receiver_node = serial_ping_pkg.acoustic_relay.smarc_pos_receiver_node:main',
            'serial_broadcast_receiver = serial_ping_pkg.common.serial_broadcast_receiver:main',
            'twtt_leader_node = serial_ping_pkg.tuper_twtt.twtt_leader_node:main',
            'twtt_follower_node = serial_ping_pkg.tuper_twtt.twtt_follower_node:main',
            'single_target_ping_node = serial_ping_pkg.common.single_target_ping_node:main',
            'owtt_follower_node = serial_ping_pkg.tuper_owtt.owtt_follower_node:main',
            'owtt_leader_node = serial_ping_pkg.tuper_owtt.owtt_leader_node:main',
            'teensy_cmd = serial_ping_pkg.tuper_owtt.teensy_cmd:main',
            'owtt_beacon_node = serial_ping_pkg.owtt_beacon.beacon_node:main',
            'owtt_surface_unit_node = serial_ping_pkg.owtt_beacon.surface_unit_node:main',
            'owtt_inference_node = serial_ping_pkg.owtt_beacon.inference_node:main',
            'owtt_inference_smoother_node = serial_ping_pkg.owtt_beacon.inference_smoother_node:main',
            'owtt_fake_mqtt = serial_ping_pkg.owtt_beacon.fake_mqtt_publisher:main',
            'ping_estimator_action_node = serial_ping_pkg.ping_estimator_action.ping_estimator_action_node:main',
            'ping_estimator_action_cmd = serial_ping_pkg.ping_estimator_action.ping_estimator_action_cmd:main',
        ],
    },
)
