"""Agent map, stick acoustic sensor, and the one-writer vehicle relay."""

import json

from serial_ping_pkg.owtt_beacon.acoustic_sensor import (
    cluster_id,
    expand_mqtt_reports,
    make_contact,
    vehicle_namespace,
)
from serial_ping_pkg.owtt_beacon.agent_map import (
    Agent,
    build_agent_tables,
    resolve_acoustic_goal,
)
from serial_ping_pkg.owtt_beacon.contact_relay import ContactRelay


def test_beacon_launch_arg_overwrites_the_yaml_modem():
    by_name, by_modem = build_agent_tables(
        {'agents': {'lolo': {'modem_id': '101', 'domain': 'subsurface'}}},
        beacon_name='lolo',
        beacon_modem_id='7',
    )
    assert by_name['lolo'].modem_id == '007'
    assert by_name['lolo'].domain == 'subsurface'
    assert '007' in by_modem
    assert '101' not in by_modem


def test_goal_resolves_an_agent_name_and_rejects_an_unknown_one():
    agents = {
        'lolo': Agent('lolo', '007', commands=['start_telemetry', 'abort']),
    }
    ok, modem, _err = resolve_acoustic_goal(
        'start_telemetry', {'target': 'lolo'}, agents)
    assert ok and modem == '007'
    ok, _modem, err = resolve_acoustic_goal(
        'stop_telemetry', {'target': 'lolo'}, agents)
    assert not ok and 'does not accept' in err
    ok, modem, _err = resolve_acoustic_goal('abort_all', {}, agents)
    assert ok and modem == ''
    ok, _modem, err = resolve_acoustic_goal(
        'abort', {'target': 'sam'}, agents)
    assert not ok and 'unknown agent' in err


def test_same_payload_clusters_together_and_a_second_agent_is_filtered():
    telem = {'position': {'lat': 58.8, 'lon': 17.6}, 'depth': 2.0, 'speed': 0.4}
    assert cluster_id('007', telem) == cluster_id('007', dict(telem))
    contact = make_contact(
        'stick_2', 'lolo', '007', 10.0, 58.82, 17.63, 40.0, 800000.0,
        771600.0, 1480.0, telem)
    other = make_contact(
        'stick_2', 'sam', '005', 10.0, 58.82, 17.63, 12.0, 100.0,
        771600.0, 1480.0, {'depth': 1.0})
    payload = json.dumps({'type': 'AcousticContacts', 'contacts': [contact, other]})
    reports = expand_mqtt_reports(
        'tuper/unit/surface/real/user_stick_2/sensor/acoustic', payload, 'lolo')
    assert len(reports) == 1
    assert reports[0]['beacon'] == 'lolo'
    assert reports[0]['range_m'] == 40.0
    legacy = expand_mqtt_reports(
        'tuper/owtt_beacon/lolo/range/stick_2',
        json.dumps({'beacon': 'lolo', 'surface_unit': 'stick_2', 'range_m': 3}),
        'lolo')
    assert legacy[0]['range_m'] == 3


def test_relay_writes_once_per_broadcast_and_stops_when_the_vehicle_is_alive():
    agent = Agent('lolo', '007')
    relay = ContactRelay({'lolo': agent}, stale_s=10.0, cluster_window_s=1.5)
    topic = 'tuper/unit/surface/real/totallynotsmarc_stick_2/sensor/acoustic'
    telem = {'position': {'lat': 58.8, 'lon': 17.6}, 'depth': 2.5, 'speed': 0.4}
    contact = make_contact(
        'stick_2', 'lolo', '007', 10.0, 58.82, 17.63, 40.0, 1.0, 1.0, 1500.0, telem)
    document = {'type': 'AcousticContacts', 'contacts': [contact]}

    first, warnings = relay.observe(topic, document, now=100.0)
    assert warnings == []
    assert [item[0] for item in first] == [
        'tuper/unit/subsurface/real/totallynotsmarc_lolo/sensor/position',
        'tuper/unit/subsurface/real/totallynotsmarc_lolo/sensor/depth',
        'tuper/unit/subsurface/real/totallynotsmarc_lolo/sensor/speed',
    ]
    again, _warnings = relay.observe(topic, document, now=100.4)
    assert again == []

    other = dict(contact)
    other['cluster'] = 'different'
    _skipped, warnings = relay.observe(
        topic, {'contacts': [other]}, now=101.0)
    assert warnings and 'disagreed' in warnings[0]

    later, _warnings = relay.observe(topic, document, now=104.0)
    assert later

    namespace = vehicle_namespace(topic, 'stick_2', agent)
    relay.note_heartbeat(
        namespace + '/heartbeat', json.dumps({'stamp': 200.0}), now=201.0)
    quiet, _warnings = relay.observe(topic, document, now=205.0)
    assert quiet == []
