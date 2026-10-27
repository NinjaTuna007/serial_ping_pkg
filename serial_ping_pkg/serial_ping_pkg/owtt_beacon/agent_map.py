"""Modem address to WARA-PS agent.

The acoustic header carries a modem id. LoLo versus SAM is this table, not
the behavior-tree string in the payload and not which stick heard the ping.
``beacon.name`` / ``beacon.modem_id`` always overwrite the entry of that
name, so an existing launch arg still retargets the one beacon.
"""

from serial_ping_pkg.common.acoustic_command import modem_int
from serial_ping_pkg.tuper_owtt import teensy_interface as ti

DEFAULT_COMMANDS = (
    'abort',
    'abort_all',
    'start_telemetry',
    'stop_telemetry',
    'enough',
)


class Agent:
    def __init__(self, name, modem_id, domain='subsurface', realsim='real',
                 commands=None, mqtt_name=''):
        self.name = str(name)
        self.modem_id = modem_id
        self.domain = domain or 'subsurface'
        self.realsim = realsim or 'real'
        self.commands = set(commands) if commands else set(DEFAULT_COMMANDS)
        self.mqtt_name = mqtt_name or ''


def _modem(value):
    if value in (None, ''):
        return ''
    number = modem_int(value)
    if number is None:
        return ''
    return ti.normalize_modem_id(number)


def _one(name, spec):
    spec = spec or {}
    modem_id = _modem(spec.get('modem_id'))
    if not modem_id:
        return None
    commands = spec.get('commands')
    if isinstance(commands, str):
        commands = [part.strip() for part in commands.split(',') if part.strip()]
    return Agent(
        name=name,
        modem_id=modem_id,
        domain=spec.get('domain') or 'subsurface',
        realsim=spec.get('realsim') or 'real',
        commands=commands,
        mqtt_name=spec.get('mqtt_name') or '',
    )


def build_agent_tables(config, beacon_name=None, beacon_modem_id=None):
    """Return ``(by_name, by_modem)``.

    A launch override of the beacon modem replaces the yaml row for
    ``beacon_name``. Two names that share a modem keep the later row.
    """
    by_name = {}
    raw = {}
    if isinstance(config, dict):
        raw = config.get('agents') or {}
    if isinstance(raw, dict):
        for name, spec in raw.items():
            agent = _one(name, spec)
            if agent is not None:
                by_name[agent.name] = agent

    overlay = _modem(beacon_modem_id)
    if beacon_name and overlay:
        previous = by_name.get(beacon_name)
        by_name[beacon_name] = Agent(
            name=beacon_name,
            modem_id=overlay,
            domain=previous.domain if previous else 'subsurface',
            realsim=previous.realsim if previous else 'real',
            commands=previous.commands if previous else None,
            mqtt_name=previous.mqtt_name if previous else '',
        )

    by_modem = {}
    for agent in by_name.values():
        by_modem[agent.modem_id] = agent
    return by_name, by_modem


def resolve_acoustic_goal(command, goal, agents_by_name):
    """Map a WARA-PS action goal onto an ``AcousticCommand`` target.

    Returns ``(ok, modem_id, error)``. ``abort_all`` has an empty target.
    ``target`` may be an agent name or a modem id.
    """
    command = '' if command is None else str(command).strip()
    if command == 'abort_all':
        return True, '', ''
    if not isinstance(goal, dict):
        return False, '', 'goal must be a JSON object'
    target = goal.get('target', goal.get('target_modem'))
    if target in (None, ''):
        return False, '', 'target agent is required'

    number = modem_int(target)
    if number is not None:
        modem_id = f'{number:03d}'
        for agent in agents_by_name.values():
            if agent.modem_id == modem_id and command not in agent.commands:
                return False, '', f'{agent.name} does not accept {command}'
        return True, modem_id, ''

    name = str(target)
    agent = agents_by_name.get(name)
    if agent is None:
        return False, '', f'unknown agent {name}'
    if command not in agent.commands:
        return False, '', f'{name} does not accept {command}'
    return True, agent.modem_id, ''
