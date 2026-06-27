"""Make the sibling ``full_stack_harness`` importable from the test files.

pytest's import machinery usually adds the test directory to ``sys.path``, but
doing it explicitly here keeps ``from full_stack_harness import ...`` working
under both ``python3 -m pytest`` and ``colcon test`` regardless of rootdir.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))


@pytest.fixture(autouse=True)
def _reap_full_stack_processes():
    """Kill any harness-spawned subprocess after each test.

    A test that fails before reaching ``Stack.stop()`` would otherwise leak an
    orphaned driver/node/socat that keeps running, pollutes the ROS graph and
    loads the machine (which in turn makes other tests flaky).
    """
    yield
    try:
        import full_stack_harness
        full_stack_harness.reap_spawned()
    except Exception:
        pass
