"""One writer for an underwater agent's sensors.

Every stick publishes the broadcast it heard. This keeps the first copy of
each broadcast and writes that agent's position, depth, and speed. A second
copy inside the cluster window is the same ping. A different payload in that
window is logged and skipped. The next broadcast, a few seconds later, is a
new write.

While the vehicle's own heartbeat is fresh, the vehicle owns those topics.
"""

import json

from serial_ping_pkg.owtt_beacon.acoustic_sensor import vehicle_namespace


def _float_or_none(value):
    if isinstance(value, bool) or value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def sensor_payloads(telemetry):
    """MQTT bodies for the standard WARA-PS sensors this payload can fill."""
    telemetry = telemetry or {}
    out = []
    position = telemetry.get('position')
    lat = lon = None
    if isinstance(position, dict):
        lat = _float_or_none(position.get('lat'))
        lon = _float_or_none(position.get('lon'))
    elif isinstance(position, (list, tuple)) and len(position) >= 2:
        lat = _float_or_none(position[0])
        lon = _float_or_none(position[1])
    if lat is not None and lon is not None:
        out.append(('sensor/position', json.dumps({
            'latitude': lat,
            'longitude': lon,
            'altitude': 0,
            'type': 'GeoPoint',
        })))
    depth = _float_or_none(telemetry.get('depth'))
    if depth is not None:
        out.append(('sensor/depth', json.dumps(depth)))
    speed = _float_or_none(telemetry.get('speed'))
    if speed is not None:
        out.append(('sensor/speed', json.dumps(speed)))
    return out


class ContactRelay:
    def __init__(self, agents_by_name, stale_s=10.0, cluster_window_s=1.5):
        self.agents_by_name = agents_by_name
        self.stale_s = float(stale_s)
        self.cluster_window_s = float(cluster_window_s)
        self._heartbeat = {}
        self._open = {}

    def note_heartbeat(self, topic, payload, now):
        if not topic.endswith('/heartbeat'):
            return
        namespace = topic[: -len('/heartbeat')]
        try:
            data = json.loads(payload)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        stamp = _float_or_none(data.get('stamp'))
        self._heartbeat[namespace] = now if stamp is None else stamp

    def heartbeat_fresh(self, namespace, now):
        stamp = self._heartbeat.get(namespace)
        if stamp is None:
            return False
        return (now - stamp) <= self.stale_s

    def observe(self, stick_topic, document, now):
        """Return ``(publishes, warnings)``.

        ``publishes`` is a list of ``(mqtt_topic, payload)``.
        """
        publishes = []
        warnings = []
        if not isinstance(document, dict):
            return publishes, warnings
        for contact in document.get('contacts') or []:
            if not isinstance(contact, dict):
                continue
            name = contact.get('agent')
            agent = self.agents_by_name.get(name)
            if agent is None:
                warnings.append(f'unknown agent {name}')
                continue
            namespace = vehicle_namespace(
                stick_topic, contact.get('surface_unit') or '', agent)
            if namespace is None:
                warnings.append(f'no namespace for {name} from {stick_topic}')
                continue
            if self.heartbeat_fresh(namespace, now):
                continue
            cluster = contact.get('cluster') or ''
            event = self._open.get(namespace)
            if event is not None and (now - event['time']) <= self.cluster_window_s:
                if event['cluster'] == cluster:
                    continue
                warnings.append(
                    f'{name} broadcast disagreed inside {self.cluster_window_s:.1f}s; kept the first')
                continue
            self._open[namespace] = {'time': now, 'cluster': cluster}
            for suffix, payload in sensor_payloads(contact.get('telemetry')):
                publishes.append((f'{namespace}/{suffix}', payload))
        return publishes, warnings
