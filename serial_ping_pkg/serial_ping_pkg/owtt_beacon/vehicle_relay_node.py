"""Copy heard telemetry onto the underwater agent's WARA-PS sensors.

Sticks publish ``sensor/acoustic``. This is the one MQTT client that writes
``sensor/position``, ``sensor/depth``, and ``sensor/speed`` on the vehicle
namespace, and only while that vehicle's heartbeat is stale.
"""

import json

from serial_ping_pkg.owtt_beacon.agent_map import build_agent_tables
from serial_ping_pkg.owtt_beacon.contact_relay import ContactRelay
from serial_ping_pkg.owtt_beacon.mqtt_helper import MqttClient
from serial_ping_pkg.utils import load_yaml_config


class VehicleRelay:
    def __init__(self, node, relay, mqtt, qos):
        self._node = node
        self._relay = relay
        self._mqtt = mqtt
        self._qos = qos

    def on_message(self, topic, payload):
        now = self._node.get_clock().now().nanoseconds * 1e-9
        if topic.endswith('/heartbeat'):
            self._relay.note_heartbeat(topic, payload, now)
            return
        try:
            document = json.loads(payload)
        except (TypeError, ValueError):
            self._node.get_logger().warn(
                f'Bad acoustic document on {topic}', throttle_duration_sec=5.0)
            return
        publishes, warnings = self._relay.observe(topic, document, now)
        for warning in warnings:
            self._node.get_logger().warn(warning, throttle_duration_sec=5.0)
        for mqtt_topic, body in publishes:
            self._mqtt.publish(mqtt_topic, body, qos=self._qos)
            self._node.get_logger().info(f'-> {mqtt_topic}')


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node('owtt_vehicle_relay')
    id_desc = ParameterDescriptor(dynamic_typing=True)
    config = load_yaml_config('serial_ping_pkg', 'owtt_beacon/owtt_beacon_config.yaml')
    beacon_cfg = config.get('beacon', {})
    mqtt_cfg = config.get('mqtt', {})
    node.declare_parameter('beacon.name', beacon_cfg.get('name', 'lolo'))
    node.declare_parameter('beacon.modem_id', beacon_cfg.get('modem_id', ''), id_desc)
    node.declare_parameter('mqtt.host', mqtt_cfg.get('host', 'localhost'))
    node.declare_parameter('mqtt.port', mqtt_cfg.get('port', 1884))
    node.declare_parameter('mqtt.keepalive', mqtt_cfg.get('keepalive', 30))
    node.declare_parameter('mqtt.username', mqtt_cfg.get('username', ''))
    node.declare_parameter('mqtt.password', mqtt_cfg.get('password', ''))
    node.declare_parameter('mqtt.qos', mqtt_cfg.get('qos', 0))
    node.declare_parameter(
        'mqtt.acoustic_topic',
        mqtt_cfg.get('acoustic_topic', '+/unit/surface/+/+/sensor/acoustic'))
    node.declare_parameter(
        'mqtt.heartbeat_topic',
        mqtt_cfg.get('heartbeat_topic', '+/unit/+/+/+/heartbeat'))
    node.declare_parameter('relay.stale_s', 10.0)
    node.declare_parameter('relay.cluster_window_s', 1.5)

    beacon_name = node.get_parameter('beacon.name').get_parameter_value().string_value
    beacon_modem = node.get_parameter('beacon.modem_id').value
    agents_by_name, _by_modem = build_agent_tables(config, beacon_name, beacon_modem)
    relay = ContactRelay(
        agents_by_name,
        stale_s=node.get_parameter('relay.stale_s').get_parameter_value().double_value,
        cluster_window_s=node.get_parameter(
            'relay.cluster_window_s').get_parameter_value().double_value,
    )
    host = node.get_parameter('mqtt.host').get_parameter_value().string_value
    port = node.get_parameter('mqtt.port').get_parameter_value().integer_value
    username = node.get_parameter('mqtt.username').get_parameter_value().string_value
    password = node.get_parameter('mqtt.password').get_parameter_value().string_value
    qos = node.get_parameter('mqtt.qos').get_parameter_value().integer_value
    acoustic_topic = node.get_parameter('mqtt.acoustic_topic').get_parameter_value().string_value
    heartbeat_topic = node.get_parameter('mqtt.heartbeat_topic').get_parameter_value().string_value

    mqtt = MqttClient(
        host, port,
        keepalive=node.get_parameter('mqtt.keepalive').get_parameter_value().integer_value,
        username=(username or None), password=(password or None),
        client_id='owtt_vehicle_relay', logger=node.get_logger())
    handler = VehicleRelay(node, relay, mqtt, qos)
    mqtt.set_message_handler(handler.on_message)
    mqtt.add_subscription(acoustic_topic)
    mqtt.add_subscription(heartbeat_topic)
    mqtt.start()
    names = ', '.join(agent.name for agent in agents_by_name.values()) or 'none'
    node.get_logger().info(
        f'Vehicle relay up for {names}. Acoustic {acoustic_topic}. '
        f'Heartbeats {heartbeat_topic}.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        mqtt.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
