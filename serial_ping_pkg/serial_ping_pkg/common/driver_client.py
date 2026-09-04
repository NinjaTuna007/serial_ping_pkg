"""Client helper for talking to ``succorfish_driver`` over ROS.

The ``succorfish_driver`` node exclusively owns the physical serial port. Every
node in ``serial_ping_pkg`` therefore no longer opens a serial port itself;
instead it uses this thin helper to:

* write raw commands by publishing ``std_msgs/String`` on ``TX_TOPIC``
  (the driver appends the configured line terminator),
* write raw UART bytes by publishing ``succorfish_msgs/SerialFrame`` on
  ``TX_BYTES_TOPIC`` (no terminator appended — use this for NM3 ``$B``/``$U``
  with a binary payload),
* receive every inbound *text* line as it arrives by subscribing to ``RX_TOPIC``
  (``succorfish_msgs/SerialLine``) and getting a per-line callback,
* receive every inbound frame as bytes on ``RX_BYTES_TOPIC`` (optional
  ``on_frame`` callback; binary ``#B``/``#U`` never appear on ``RX_TOPIC``),
* observe link state via ``STATUS_TOPIC`` (``std_msgs/Bool``, latched),
* perform synchronous request/response with the ``SendCommand`` service
  (write a command, wait for a reply line matching a regex within a timeout).

The driver is protocol-agnostic aside from NM3 length-prefixed ``#B``/``#U``
framing needed to deliver vendor-legal binary payloads. Application codecs
(``$P``, timestamp envelope, DCCL, ...) stay client-side.

Topic and service names come from ``succorfish_msgs/Topics`` and are *relative*
(``succorfish/...``), so a node and its driver line up automatically when they
share a namespace. Remap or namespace them together to address a specific driver
(e.g. the 9600-baud Succorfish vs. the 115200-baud Teensy profile).
"""

from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import Bool, String
from succorfish_msgs.msg import SerialFrame, SerialLine, Topics
from succorfish_msgs.srv import SendCommand


class DriverClient:
    """ROS-side facade over the ``succorfish_driver`` serial bridge.

    Parameters
    ----------
    node:
        The owning ``rclpy`` node; publishers/subscriptions/clients are created
        on it.
    on_line:
        Optional ``callable(line: str)`` invoked for every inbound *text*
        serial line. When ``None`` no RX subscription is created (write-only
        nodes). Binary ``#B``/``#U`` frames are not delivered here.
    on_frame:
        Optional ``callable(data: bytes, stamp)`` invoked for every inbound
        frame on ``RX_BYTES_TOPIC`` (text and binary). When ``None`` no bytes
        subscription is created.
    on_status:
        Optional ``callable(connected: bool)`` invoked on link-state changes
        (latched, so it fires once on startup with the current state).
    callback_group:
        Optional callback group for the subscription/service client.
    """

    def __init__(self, node, on_line=None, on_status=None, on_frame=None,
                 rx_topic=Topics.RX_TOPIC, tx_topic=Topics.TX_TOPIC,
                 rx_bytes_topic=Topics.RX_BYTES_TOPIC,
                 tx_bytes_topic=Topics.TX_BYTES_TOPIC,
                 status_topic=Topics.STATUS_TOPIC,
                 service_name=Topics.SEND_COMMAND_SERVICE,
                 shutdown_topic=Topics.SHUTDOWN_COMMAND_TOPIC,
                 callback_group=None):
        self.node = node
        self._on_line = on_line
        self._on_frame = on_frame
        self._cbg = callback_group

        self.tx_pub = node.create_publisher(String, tx_topic, 10)
        self.tx_bytes_pub = node.create_publisher(
            SerialFrame, tx_bytes_topic, 10)

        if on_line is not None:
            node.create_subscription(
                SerialLine, rx_topic, self._rx_cb, 10, callback_group=callback_group)

        if on_frame is not None:
            node.create_subscription(
                SerialFrame, rx_bytes_topic, self._rx_bytes_cb, 10,
                callback_group=callback_group)

        if on_status is not None:
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(
                Bool, status_topic, lambda m: on_status(m.data), latched,
                callback_group=callback_group)

        self.cmd_client = node.create_client(
            SendCommand, service_name, callback_group=callback_group)

        self._shutdown_topic = shutdown_topic
        self._shutdown_pub = None

    # -- inbound -------------------------------------------------------------

    def _rx_cb(self, msg):
        self._on_line(msg.line)

    def _rx_bytes_cb(self, msg):
        self._on_frame(bytes(msg.data), msg.stamp)

    # -- outbound ------------------------------------------------------------

    def write(self, command):
        """Fire-and-forget: publish a raw command on ``TX_TOPIC``.

        The driver appends its configured terminator (default ``\\r\\n``) before
        writing to the wire, so callers pass the bare command (``$B12...``,
        ``$G...``, ``$YW`` ...). Do not use this for a payload that contains
        non-UTF-8 bytes; use ``write_bytes`` instead.
        """
        self.tx_pub.publish(String(data=command))

    def write_bytes(self, data):
        """Fire-and-forget: publish raw UART bytes on ``TX_BYTES_TOPIC``.

        The driver writes ``data`` as-is (no terminator). Use this for NM3
        ``$B``/``$U``/``$M`` with a binary payload. ``data`` is ``bytes`` or
        any buffer of uint8.
        """
        msg = SerialFrame()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.data = bytes(data)
        self.tx_bytes_pub.publish(msg)

    def request(self, command, expect_regex='', timeout=0.0,
                append_terminator=True, on_result=None):
        """Synchronous-style request/response via the ``SendCommand`` service.

        Issues an async service call (so the caller never blocks the executor)
        and, when ``on_result`` is given, invokes it with the
        ``SendCommand.Response`` once the driver replies. On any call failure
        ``on_result(None)`` is invoked so callers have a single error path.

        ``expect_regex`` is matched (per line) against reply lines that arrive
        after the command is written; ``timeout`` bounds the wait. Returns the
        ``rclpy`` future.
        """
        req = SendCommand.Request()
        req.command = command
        req.append_terminator = append_terminator
        req.expect_regex = expect_regex
        req.timeout = float(timeout)
        future = self.cmd_client.call_async(req)
        if on_result is not None:
            def _done(fut):
                try:
                    result = fut.result()
                except Exception:  # noqa: BLE001 - surface as a None result
                    result = None
                on_result(result)
            future.add_done_callback(_done)
        return future

    def register_shutdown_command(self, command):
        """Register a command for the driver to write on its own graceful exit.

        Publishes (latched) to ``SHUTDOWN_COMMAND_TOPIC``. The driver replays this
        opaque string to the wire just before it closes the port, so the action
        (e.g. an OWTT node's ``$Y<id>W`` wire-mode reset) is guaranteed regardless
        of shutdown ordering -- the driver is the last holder of the open port.
        Call once at startup. The driver stays protocol-agnostic; the meaning of
        the string lives entirely with the registering node.
        """
        if self._shutdown_pub is None:
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._shutdown_pub = self.node.create_publisher(
                String, self._shutdown_topic, latched)
        self._shutdown_pub.publish(String(data=command))

    def wait_for_driver(self, timeout_sec=None):
        """Block until the ``SendCommand`` service is available (or timeout)."""
        return self.cmd_client.wait_for_service(timeout_sec=timeout_sec)
