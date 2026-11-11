"""Collecting device diagnostics must not load models or exit pytest."""
import runpy
from pathlib import Path

import pytest


@pytest.mark.parametrize('name', ['test_encoding_engine.py', 'test_sqlite_vec.py'])
def test_device_diagnostics_are_import_safe(name, capsys):
    path = Path(__file__).resolve().parents[1] / 'AuRoRA/tests' / name
    result = runpy.run_path(str(path))
    assert callable(result['main'])
    assert capsys.readouterr().out == ''
