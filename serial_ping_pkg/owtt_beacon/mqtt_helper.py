"""Small, version-robust paho-mqtt wrapper used by the OWTT beacon nodes.

Both the surface units (publish range reports) and the inference node
(subscribe to them) talk to an MQTT broker directly, mirroring the smarc2
``str_json_mqtt_bridge`` convention of sending JSON strings as the payload.

This wraps paho-mqtt so that:
  * it works with both paho-mqtt 1.x and 2.x (callback API v1),
  * connection is non-blocking and auto-reconnects (``connect_async`` +
    ``loop_start``), so nodes start even if the broker is down,
  * subscriptions are re-applied on every (re)connect.
"""

import paho.mqtt.client as mqtt


def _make_client(client_id):
    """Construct an mqtt.Client using the v1 callback API on paho 1.x or 2.x."""
    try:
        # paho-mqtt >= 2.0 requires an explicit callback API version.
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    except (AttributeError, TypeError):
        return mqtt.Client(client_id=client_id)


class MqttClient:
    """Thin convenience wrapper around an auto-reconnecting paho client."""

    def __init__(self, host, port, keepalive=30, username=None, password=None,
                 client_id="", logger=None):
        self.host = host
        self.port = int(port)
        self.keepalive = int(keepalive)
        self.logger = logger
        self.connected = False

        self._subscriptions = []          # list of (topic, qos)
        self._message_handler = None      # callable(topic, payload_str)

        self._client = _make_client(client_id)
        if username:
            self._client.username_pw_set(username, password or None)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

    # ------------------------------------------------------------------ logging
    def _log(self, msg, level='info'):
        if self.logger is None:
            return
        getattr(self.logger, level, self.logger.info)(f"[mqtt] {msg}")

    # ------------------------------------------------------------------ lifecycle
    def set_message_handler(self, handler):
        """Register ``handler(topic: str, payload: str)`` for incoming messages."""
        self._message_handler = handler

    def add_subscription(self, topic, qos=0):
        """Register a subscription; applied now if connected, else on connect."""
        self._subscriptions.append((topic, qos))
        if self.connected:
            self._client.subscribe(topic, qos)

    def start(self):
        """Begin connecting (non-blocking) and run the network loop in a thread."""
        self._log(f"connecting to {self.host}:{self.port}")
        try:
            self._client.connect_async(self.host, self.port, self.keepalive)
        except Exception as e:  # pragma: no cover - DNS/socket setup
            self._log(f"connect_async failed: {e}", 'warn')
        self._client.loop_start()

    def stop(self):
        try:
            self._client.disconnect()
        except Exception:
            pass
        try:
            self._client.loop_stop()
        except Exception:
            pass

    def publish(self, topic, payload, qos=0, retain=False):
        return self._client.publish(topic, payload, qos=qos, retain=retain)

    # ------------------------------------------------------------------ callbacks
    def _on_connect(self, client, userdata, flags, rc):
        self.connected = (rc == 0)
        if rc == 0:
            self._log("connected")
            for topic, qos in self._subscriptions:
                client.subscribe(topic, qos)
        else:
            self._log(f"connect failed (rc={rc})", 'warn')

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        self._log(f"disconnected (rc={rc}); will auto-reconnect", 'warn')

    def _on_message(self, client, userdata, msg):
        if self._message_handler is None:
            return
        try:
            payload = msg.payload.decode('utf-8', errors='ignore')
        except Exception:
            return
        try:
            self._message_handler(msg.topic, payload)
        except Exception as e:  # never let a bad message kill the loop thread
            self._log(f"message handler error on {msg.topic}: {e}", 'error')
