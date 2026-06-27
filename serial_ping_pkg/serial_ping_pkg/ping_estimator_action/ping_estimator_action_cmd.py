"""Goal-sending CLI for the ping estimator action server.

The ``ping_estimator_action_node`` exposes a four-mode action server
(``smarc_modem_ping``) built on ``smarc_msgs/action/BaseAction``. The goal is a
``std_msgs/String`` whose ``data`` is a JSON command:

    {"mode": "add",    "modem_id": "7", "depth_m": 12.0}
    {"mode": "remove", "modem_id": "7"}
    {"mode": "clear"}
    {"mode": "ping",   "retry_count": 3, "task_timeout_s": 180.0}
    {"mode": "broadcast", "message": "hello all"}
    {"mode": "unicast",   "modem_id": "8", "message": "hello modem 8"}

This tool builds that JSON, sends it to the action server, streams feedback, and
prints the result. The action lives under the node namespace (the robot name),
so the default server is ``/lolo/smarc_modem_ping``.

Examples (run via ``ros2 run serial_ping_pkg ping_estimator_action_cmd``):

    # Add modem 7 with a known depth of 12 m, then ping/localize everything
    ping_estimator_action_cmd add 7 --depth 12.0
    ping_estimator_action_cmd ping --retries 3 --timeout 180

    # Different vehicle namespace
    ping_estimator_action_cmd add 7 --robot sam

    # Broadcast messages
    ping_estimator_action_cmd broadcast "hello all"
    ping_estimator_action_cmd unicast 8 "hello modem 8"

    # Remove one modem / clear them all / hard-stop the running ping task
    ping_estimator_action_cmd remove 7
    ping_estimator_action_cmd clear
    ping_estimator_action_cmd stop
"""

import argparse
import json
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from smarc_msgs.action import BaseAction

DEFAULT_PAYLOAD_MAX_BYTES = 64

def _message_from_cli(parts: list[str], max_bytes: int) -> str:
    """Build a payload message from CLI words and check the local byte limit."""
    message = " ".join(parts)

    if "\r" in message or "\n" in message:
        raise SystemExit("payload message must not contain CR/LF")

    byte_count = len(message.encode("utf-8"))
    if byte_count > max_bytes:
        raise SystemExit(
            f"payload too large: {byte_count} bytes > {max_bytes} bytes"
        )

    return message


def _build_goal_payload(args) -> dict:
    """Turn parsed CLI args into the JSON command dict the node expects."""
    mode = args.mode

    if mode in ('add', 'remove'):
        payload = {'mode': mode, 'modem_id': str(int(args.modem_id))}
        if mode == 'add' and args.depth is not None:
            payload['depth_m'] = float(args.depth)
        return payload

    if mode == 'clear':
        return {'mode': 'clear'}

    if mode == 'ping':
        return {
            'mode': 'ping',
            'retry_count': int(args.retries),
            'task_timeout_s': float(args.timeout),
        }

    if mode == 'broadcast':
        return {
            'mode': 'broadcast',
            'message': _message_from_cli(args.message, args.payload_max_bytes),
        }

    if mode == 'unicast':
        return {
            'mode': 'unicast',
            'modem_id': str(int(args.modem_id)),
            'message': _message_from_cli(args.message, args.payload_max_bytes),
        }

    raise SystemExit(f"Unknown mode: {mode!r}")


def _send_goal(node: Node, action_name: str, payload: dict, wait: bool) -> bool:
    """Send one BaseAction goal and (optionally) wait for the result."""
    client = ActionClient(node, BaseAction, action_name)

    node.get_logger().info(f"waiting for action server {action_name} ...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f"action server {action_name} not available")
        return False

    goal = BaseAction.Goal()
    goal.goal.data = json.dumps(payload)
    node.get_logger().info(f"-> {action_name}: {goal.goal.data}")

    def _feedback_cb(feedback_msg):
        node.get_logger().info(f"feedback: {feedback_msg.feedback.feedback.data}")

    send_future = client.send_goal_async(goal, feedback_callback=_feedback_cb)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()

    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("goal rejected by server")
        return False

    node.get_logger().info("goal accepted")

    if not wait:
        return True

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()

    if result is None:
        node.get_logger().error("no result returned")
        return False

    success = bool(result.result.success)
    node.get_logger().info(f"result: success={success}")
    return success


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='ping_estimator_action_cmd',
        description="Send add/remove/clear/ping/broadcast/unicast/stop goals to the modem ping estimator.")
    parser.add_argument('--robot', default='lolo',
                        help="Robot namespace the node runs in (default %(default)s).")
    parser.add_argument('--action', default=None,
                        help="Full action name override; bypasses --robot "
                             "(e.g. /lolo/smarc_modem_ping).")
    parser.add_argument('--no-wait', action='store_true',
                        help="Send the goal and exit without waiting for the result.")
    
    parser.add_argument('--payload-max-bytes', type=int, default=DEFAULT_PAYLOAD_MAX_BYTES,
                        help="Local max payload size check for broadcast/unicast "
                             "(default %(default)s). The server still enforces its own limit.")

    sub = parser.add_subparsers(dest='mode', required=True)

    p_add = sub.add_parser('add', help="Add a modem id (optionally with known depth).")
    p_add.add_argument('modem_id', help="Modem id, 0-255 (e.g. 7).")
    p_add.add_argument('--depth', type=float, default=None,
                       help="Known modem depth in meters, positive down.")

    p_remove = sub.add_parser('remove', help="Remove one modem id.")
    p_remove.add_argument('modem_id', help="Modem id to remove.")

    sub.add_parser('clear', help="Remove all modem ids and delete markers.")

    p_ping = sub.add_parser('ping', help="Ping/localize all active modem ids.")
    p_ping.add_argument('--retries', type=int, default=3,
                        help="Retries after the first attempt (default %(default)s).")
    p_ping.add_argument('--timeout', type=float, default=180.0,
                        help="Overall task timeout in seconds (default %(default)s).")
    

    p_broadcast = sub.add_parser('broadcast', help="Send a broadcast acoustic message.")
    p_broadcast.add_argument(
        'message',
        nargs='+',
        help="Message payload. Quotes preserve spaces exactly; otherwise words are joined with spaces.",
    )

    p_unicast = sub.add_parser('unicast', help="Send a unicast acoustic message to one modem id.")
    p_unicast.add_argument('modem_id', help="Destination modem id, 0-255.")
    p_unicast.add_argument(
        'message',
        nargs='+',
        help="Message payload. Quotes preserve spaces exactly; otherwise words are joined with spaces.",
    )

    sub.add_parser('stop', help="Hard-stop: clears the list via the stop action server.")

    args = parser.parse_args(argv)

    if args.action is not None:
        action_name = args.action
    else:
        ns = args.robot.strip('/')
        base = 'smarc_stop_modem_ping' if args.mode == 'stop' else 'smarc_modem_ping'
        action_name = f"/{ns}/{base}" if ns else f"/{base}"

    if args.mode == 'stop':
        payload = {'mode': 'clear'}
    else:
        payload = _build_goal_payload(args)

    rclpy.init()
    node = rclpy.create_node('ping_estimator_action_cmd')
    try:
        ok = _send_goal(node, action_name, payload, wait=not args.no_wait)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
