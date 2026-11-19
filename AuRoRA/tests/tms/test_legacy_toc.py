"""Legacy AuRoRA output adapter still needs valid connection/lifecycle wiring."""
import asyncio
import sys
import json
from pathlib import Path
from unittest.mock import Mock


AURO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AURO_ROOT / 'src/tms'))
sys.path.insert(0, str(AURO_ROOT / 'src/hrs'))
sys.path.insert(0, str(AURO_ROOT / 'src/gms'))


def test_output_port_and_connection_lifetime():
    from tms.toc import TOC, TMS
    assert TMS.TELEEFFECTOR_PORTAL == 8851
    node = TOC.__new__(TOC)
    node._active_connections = set()
    node.get_logger = Mock(return_value=Mock())
    class Link:
        async def wait_closed(self):
            assert self in node._active_connections
    link = Link()
    asyncio.run(node._on_peripheral_connected(link))
    assert not node._active_connections


def test_broadcast_survives_connections_leaving_during_send():
    from tms.toc import TOC
    node = TOC.__new__(TOC)
    node.get_logger = Mock(return_value=Mock())
    sent = []
    class Link:
        async def send(self, raw):
            sent.append(raw)
            node._active_connections.discard(self)
    node._active_connections = {Link(), Link()}
    asyncio.run(node._broadcast({'type': 'done', 'content': 'hello'}))
    assert len(sent) == 2


def test_input_buffer_and_publish_use_actual_sss_content():
    from tms.tic import TelepathyInputCore
    from gms.csb import SSS, SensoryInputChannel, TraceType
    node = TelepathyInputCore.__new__(TelepathyInputCore)
    node.get_logger = Mock(return_value=Mock())
    node._stimulus_gateway = Mock()
    signal = SSS(content='great thanks', source=SensoryInputChannel.UI, trace_type=TraceType.PMT)
    node._buffer(signal)
    node._publish_sss(signal)
    payload = json.loads(node._stimulus_gateway.publish.call_args.args[0].data)
    assert payload['text'] == 'great thanks'
    assert payload['valence'] > 0


def test_input_echo_rejects_non_object_and_nonfinite_duration():
    from tms.tic import TelepathyInputCore
    node = TelepathyInputCore.__new__(TelepathyInputCore)
    node.get_logger = Mock(return_value=Mock())
    node._response_echoes = {}
    for raw in ('null', '[]', '"text"', '{"imprint":"abc","duration":"NaN"}', '{"imprint":123,"duration":1}'):
        node._on_response_echo(type('Message', (), {'data': raw})())
    assert not node._response_echoes
    assert node.get_logger().warning.call_count == 5
