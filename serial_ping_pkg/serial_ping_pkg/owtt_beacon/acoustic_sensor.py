"""One stick sensor for every underwater agent that stick can hear.

The document is a ``std_msgs/String`` of JSON on ``waraps/sensor/acoustic``.
The bridge forwards that one topic. Each contact names the agent, so a new
vehicle is another entry, not another yaml line.
"""

import hashlib
import json
import re

ACOUSTIC_TOPIC = 'waraps/sensor/acoustic'
DOCUMENT_TYPE = 'AcousticContacts'

_STICK_TOPIC = re.compile(
    r'([^/]+)/unit/(air|surface|subsurface|command)/(real|simulation|command)/([^/]+)(/.*)$'
)


def cluster_id(modem_id, telemetry):
    """Stable id for one decoded payload. The same broadcast hashes the same on every stick."""
    body = json.dumps(telemetry or {}, sort_keys=True, separators=(',', ':'), default=str)
    digest = hashlib.sha1(f'{modem_id}|{body}'.encode('utf-8')).hexdigest()
    return digest[:12]


def make_contact(surface_unit, agent_name, modem_id, stamp, unit_lat, unit_lon,
                 range_m, delta_us, offset_us, sound_velocity, telemetry):
    telem = telemetry or {}
    return {
        'agent': agent_name,
        'modem_id': modem_id,
        'surface_unit': surface_unit,
        'stamp': stamp,
        'unit_lat': unit_lat,
        'unit_lon': unit_lon,
        'range_m': range_m,
        'delta_us': delta_us,
        'offset_us': offset_us,
        'sound_velocity': sound_velocity,
        'cluster': cluster_id(modem_id, telem),
        'telemetry': telem,
    }


def acoustic_document(contacts):
    return {'type': DOCUMENT_TYPE, 'contacts': list(contacts)}


def report_from_contact(contact):
    """Shape the inference nodes already store for one range report."""
    return {
        'surface_unit': contact.get('surface_unit'),
        'beacon': contact.get('agent'),
        'beacon_modem_id': contact.get('modem_id'),
        'stamp': contact.get('stamp'),
        'unit_lat': contact.get('unit_lat'),
        'unit_lon': contact.get('unit_lon'),
        'range_m': contact.get('range_m'),
        'delta_us': contact.get('delta_us'),
        'offset_us': contact.get('offset_us'),
        'sound_velocity': contact.get('sound_velocity'),
        'telemetry': contact.get('telemetry') or {},
    }


def expand_mqtt_reports(topic, payload, beacon_name):
    """Range reports for ``beacon_name`` inside one MQTT payload.

    A stick ``sensor/acoustic`` document can carry several agents. A legacy
    ``tuper/owtt_beacon/.../range/...`` payload is one report. ``None`` means
    the payload was not JSON.
    """
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return []
    if data.get('type') == DOCUMENT_TYPE or topic.endswith('/sensor/acoustic'):
        reports = []
        for contact in data.get('contacts') or []:
            if not isinstance(contact, dict):
                continue
            if beacon_name and contact.get('agent') not in (None, '', beacon_name):
                continue
            reports.append(report_from_contact(contact))
        return reports
    beacon = data.get('beacon')
    if beacon_name and beacon not in (None, '', beacon_name):
        return []
    return [data]


def vehicle_namespace(stick_topic, surface_unit, agent):
    """WARA-PS namespace of the underwater agent, copied from the stick topic's user prefix.

    ``tuper/unit/surface/real/totallynotsmarc_stick_2/sensor/acoustic`` and
    agent ``lolo`` become ``tuper/unit/subsurface/real/totallynotsmarc_lolo``.
    ``agent.mqtt_name`` replaces that last segment when the vehicle was
    launched under a different account.
    """
    match = _STICK_TOPIC.match(stick_topic or '')
    if match is None or agent is None:
        return None
    context, _domain, realsim, mqtt_agent, _rest = match.groups()
    if agent.mqtt_name:
        segment = agent.mqtt_name
    else:
        prefix = ''
        if surface_unit and mqtt_agent.endswith('_' + surface_unit):
            prefix = mqtt_agent[:-(len(surface_unit))]
        elif surface_unit and mqtt_agent == surface_unit:
            prefix = ''
        else:
            prefix = ''
        segment = f'{prefix}{agent.name}'
    domain = agent.domain or 'subsurface'
    mode = agent.realsim or realsim
    return f'{context}/unit/{domain}/{mode}/{segment}'
