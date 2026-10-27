"""WARA-PS actions that send one acoustic command.

Each verb is its own action server. ``wasp_bt`` lists the name in
``tasks-available`` and a ``start-task`` on that stick is the transmitter.
The goal JSON names the underwater agent. This process also publishes
vehicle-health READY, which the behavior tree requires before it will run
a task. The modem work stays in ``acoustic_cmd_node``.
"""

from std_msgs.msg import Int8

from serial_ping_interfaces.srv import AcousticCommand
from serial_ping_pkg.owtt_beacon.agent_map import (
    build_agent_tables,
    resolve_acoustic_goal,
)
from serial_ping_pkg.utils import load_yaml_config

VERBS = (
    ('acoustic_abort', 'abort'),
    ('acoustic_abort_all', 'abort_all'),
    ('acoustic_start_telemetry', 'start_telemetry'),
    ('acoustic_stop_telemetry', 'stop_telemetry'),
    ('acoustic_enough', 'enough'),
)

HEALTH_READY = 0


class AcousticVerb:
    def __init__(self, node, action_name, command, agents_by_name, client, server_cls):
        self._node = node
        self._command = command
        self._agents = agents_by_name
        self._client = client
        self._target = ''
        self._future = None
        self._sent = False
        self._server = server_cls(
            node,
            action_name,
            self._on_goal,
            self._on_cancel,
            self._prepare,
            self._loop,
            self._feedback,
            loop_frequency=2.0,
        )

    def _on_goal(self, goal):
        ok, target, error = resolve_acoustic_goal(self._command, goal, self._agents)
        if not ok:
            self._node.get_logger().error(error)
            return False
        self._target = target
        return True

    def _on_cancel(self):
        self._future = None
        self._sent = False
        return True

    def _prepare(self):
        self._future = None
        self._sent = False

    def _loop(self):
        if not self._sent:
            if not self._client.service_is_ready():
                return None
            request = AcousticCommand.Request()
            request.command = self._command
            request.target = self._target
            self._future = self._client.call_async(request)
            self._sent = True
            return None
        if self._future is None or not self._future.done():
            return None
        response = self._future.result()
        if response is None:
            return False
        self._node.get_logger().info(response.message)
        return bool(response.success)

    def _feedback(self):
        if not self._sent:
            return f'waiting for acoustic_command ({self._command})'
        if self._future is not None and not self._future.done():
            return f'{self._command} in flight, target {self._target or "all"}'
        return self._command


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    try:
        from smarc_action_base.gentler_action_server import GentlerActionServer
    except ImportError as exc:
        raise SystemExit(
            'acoustic_action_node needs smarc_action_base (GentlerActionServer) '
            f'from the smarc2 workspace: {exc}') from exc

    rclpy.init(args=args)
    node = Node('acoustic_action_node')
    id_desc = ParameterDescriptor(dynamic_typing=True)
    config = load_yaml_config('serial_ping_pkg', 'owtt_beacon/owtt_beacon_config.yaml')
    beacon_cfg = config.get('beacon', {})
    node.declare_parameter('beacon.name', beacon_cfg.get('name', 'lolo'))
    node.declare_parameter('beacon.modem_id', beacon_cfg.get('modem_id', ''), id_desc)
    node.declare_parameter('health_topic', 'smarc/vehicle_health')
    node.declare_parameter('command_service', 'acoustic_command')

    beacon_name = node.get_parameter('beacon.name').get_parameter_value().string_value
    beacon_modem = node.get_parameter('beacon.modem_id').value
    agents_by_name, _by_modem = build_agent_tables(config, beacon_name, beacon_modem)
    health_topic = node.get_parameter('health_topic').get_parameter_value().string_value
    service_name = node.get_parameter('command_service').get_parameter_value().string_value

    health_pub = node.create_publisher(Int8, health_topic, 10)

    def _pulse_health():
        msg = Int8()
        msg.data = HEALTH_READY
        health_pub.publish(msg)

    node.create_timer(1.0, _pulse_health)
    _pulse_health()

    group = ReentrantCallbackGroup()
    client = node.create_client(AcousticCommand, service_name, callback_group=group)
    verbs = [
        AcousticVerb(node, action_name, command, agents_by_name, client, GentlerActionServer)
        for action_name, command in VERBS
    ]
    del verbs
    names = ', '.join(agent.name for agent in agents_by_name.values()) or 'none'
    node.get_logger().info(
        f'Acoustic actions up for {names}. Health on {health_topic}. '
        f'Service {service_name}.')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
