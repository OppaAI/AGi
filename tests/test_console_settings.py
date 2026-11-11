"""Settings writes must preserve ROS selector boundaries."""
import importlib

import pytest
import yaml
from fastapi.testclient import TestClient


@pytest.fixture
def settings_client(tmp_path, monkeypatch):
    server = importlib.import_module('telepathic_management.telepathic_console.server')
    path = tmp_path / 'setpoints.yaml'
    monkeypatch.setattr(server, '_settings_path', lambda: path)
    with TestClient(server.app) as client:
        yield client, path, server


def test_saving_does_not_promote_node_private_parameters(settings_client):
    client, path, _ = settings_client
    path.write_text(yaml.safe_dump({'/camera': {'ros__parameters': {'device': '/dev/video1'}},
                                   '/**': {'ros__parameters': {'existing_global': 7}}}))
    response = client.put('/api/settings', json={'settings': {'scs.gce.response_depth': 100}})
    assert response.status_code == 200, response.text
    saved = yaml.safe_load(path.read_text())
    assert saved['/camera']['ros__parameters'] == {'device': '/dev/video1'}
    assert saved['/**']['ros__parameters'] == {'existing_global': 7, 'scs.gce.response_depth': 100}


def test_displayed_global_values_do_not_depend_on_yaml_order(settings_client):
    client, path, _ = settings_client
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'scs.gce.response_depth': 100}},
                                   '/worker': {'ros__parameters': {'scs.gce.response_depth': 200}}}, sort_keys=False))
    response = client.get('/api/settings')
    assert response.status_code == 200
    values = {entry['key']: entry['value'] for entry in response.json()['settings']}
    assert values['scs.gce.response_depth'] == 100
    assert client.put('/api/settings', json={'settings': {'scs.gce.response_depth': 300}}).status_code == 200
    saved = yaml.safe_load(path.read_text())
    assert saved['/worker']['ros__parameters']['scs.gce.response_depth'] == 300


def test_failed_replace_preserves_file_and_removes_temporary_file(settings_client, monkeypatch):
    client, path, server = settings_client
    path.write_text('{}\n')
    def fail(*_):
        raise OSError('simulated disk failure')
    monkeypatch.setattr(server.os, 'replace', fail)
    response = client.put('/api/settings', json={'settings': {'scs.gce.response_depth': 100}})
    assert response.status_code == 500
    assert path.read_text() == '{}\n'
    assert list(path.parent.iterdir()) == [path]
