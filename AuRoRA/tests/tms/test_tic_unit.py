"""Exercise the actual TIC/SSS pipeline; ROS-only doubles are fixture scoped."""
import importlib.util
import json
import sys
import types
from pathlib import Path
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def tic(monkeypatch):
    for package in ('tms', 'hrs', 'gms'):
        monkeypatch.syspath_prepend(str(ROOT / 'AuRoRA/src' / package))
    monkeypatch.syspath_prepend(str(ROOT / 'entity/cryptic_signal_core'))
    for name in ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg', 'cryptic_signal_core.msg'):
        if name not in sys.modules:
            monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', type('Node', (), {}), raising=False)
    monkeypatch.setattr(sys.modules['std_msgs.msg'], 'String', type('String', (), {}), raising=False)
    monkeypatch.setattr(sys.modules['cryptic_signal_core.msg'], 'TelepathicSignal', type('TelepathicSignal', (), {}), raising=False)
    # Load under a private name to avoid leaving a Node subclass bound to doubles
    # in the shared tms import cache after this fixture exits.
    spec = importlib.util.spec_from_file_location('_tic_under_test', ROOT / 'AuRoRA/src/tms/tms/tic.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.TelepathyInputCore.__new__(module.TelepathyInputCore)
    node.get_logger = Mock(return_value=Mock())
    node._stimulus_gateway = Mock()
    node._response_echoes = {}
    node._refractory_anchor = {}
    return node, module


@pytest.mark.parametrize('raw', ['null', '[]', '"hello"', '{bad'])
def test_rejects_non_object_envelopes(tic, raw):
    assert tic[0]._receive_stimulus(raw) is None


@pytest.mark.parametrize('content', ['', '  ', None, 17, []])
def test_transduction_rejects_invalid_content(tic, content):
    node, _ = tic
    signal = node._receive_stimulus(json.dumps({'content': content}))
    assert node._apply_transduction_gate(signal) is None
    assert signal.drop_reason == 'BELOW_THRESHOLD'


def test_pipeline_normalizes_and_publishes_real_content(tic):
    node, _ = tic
    signal = node._receive_stimulus(json.dumps({'content': '  great thanks  ', 'user_id': 'oppa'}))
    assert node._apply_transduction_gate(signal) is signal
    assert node._apply_heuristic_gate(signal) is signal
    node._publish_sss(node._buffer(signal))
    payload = json.loads(node._stimulus_gateway.publish.call_args.args[0].data)
    assert payload['text'] == 'great thanks'
    assert payload['user_id'] == 'oppa'
    assert payload['valence'] > 0
    assert payload['radio_id']


def test_duplicate_suppression_is_per_user(tic):
    node, _ = tic
    def stimulus(user):
        return node._receive_stimulus(json.dumps({'content': 'Hello', 'user_id': user}))
    assert node._apply_heuristic_gate(stimulus('one')) is not None
    duplicate = stimulus('one')
    assert node._apply_heuristic_gate(duplicate) is None
    assert duplicate.drop_reason == 'DUPLICATE'
    assert node._apply_heuristic_gate(stimulus('two')) is not None


def test_web_directive_passes_but_local_command_does_not(tic):
    node, _ = tic
    for content, passes in [('/web autumn leaves', True), ('/status', False)]:
        signal = node._receive_stimulus(json.dumps({'content': content}))
        assert (node._apply_heuristic_gate(signal) is not None) == passes


def test_oversize_counts_utf8_bytes(tic):
    node, module = tic
    signal = node._receive_stimulus(json.dumps({'content': '🌸' * (module.TMS.MAX_PAYLOAD_BYTES // 4 + 1)}))
    assert node._apply_heuristic_gate(signal) is None
    assert signal.drop_reason == 'OVERSIZE'


def test_echo_expiry_and_malformed_echo(tic, monkeypatch):
    node, module = tic
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100)
    imprint = node._derive_imprint('Hello')
    node._on_response_echo(types.SimpleNamespace(data=json.dumps({'imprint': imprint, 'duration': 2})))
    assert node._apply_heuristic_gate(node._receive_stimulus('{"content":"Hello"}')) is None
    monkeypatch.setattr(module.time, 'monotonic', lambda: 103)
    node._prune_response_echoes()
    assert not node._response_echoes
    for raw in ['null', '[]', '{"imprint":"a","duration":"NaN"}']:
        node._on_response_echo(types.SimpleNamespace(data=raw))
    assert not node._response_echoes


def test_publish_failure_is_logged(tic):
    node, _ = tic
    node._stimulus_gateway.publish.side_effect = RuntimeError('offline')
    node._publish_sss(node._buffer(node._receive_stimulus('{"content":"Hello"}')))
    node.get_logger().error.assert_called_once()


@pytest.mark.parametrize('payload', [{'content':'hello','user_id':[]}, {'content':'hello','radio_id':{}}])
def test_invalid_sender_fields_cannot_reach_gates(tic, payload):
    assert tic[0]._receive_stimulus(json.dumps(payload)) is None


def test_preserves_correlation_id_and_stamps_external_role(tic):
    signal = tic[0]._receive_stimulus('{"content":"hello","radio_id":"request-1","role":"system"}')
    assert signal.sss_id == 'request-1'
    assert signal.role == 'user'
