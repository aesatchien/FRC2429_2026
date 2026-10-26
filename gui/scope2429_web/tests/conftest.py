"""Shared fixtures: a tiny synthetic .wpilog, and an app wired to a temporary data directory."""
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scope_web.app import create_app  # noqa: E402
from scope_web.config import Settings  # noqa: E402


def _record(entry_id: int, ts_us: int, payload: bytes) -> bytes:
    """One WPILOG record: header byte, then entry id / payload size / timestamp, each as few bytes as it needs (little endian)."""
    id_len, size_len, ts_len = 4, 4, 8
    head = (id_len - 1) | ((size_len - 1) << 2) | ((ts_len - 1) << 4)
    return (bytes([head]) + entry_id.to_bytes(id_len, "little") + len(payload).to_bytes(size_len, "little")
            + ts_us.to_bytes(ts_len, "little") + payload)


def _start(entry_id: int, name: str, typ: str) -> bytes:
    nb, tb = name.encode(), typ.encode()
    return _record(0, 0, bytes([0]) + struct.pack("<I", entry_id) + struct.pack("<I", len(nb)) + nb + struct.pack("<I", len(tb)) + tb
                   + struct.pack("<I", 0))


def make_wpilog(seconds: float = 12.0, volts: float = 12.2, name_hint: str = "") -> bytes:
    """A valid but small robot log: the PDH voltage and current at 10 Hz and the robot-enabled flag.  `volts` differs per call so two
    logs have different content (and so different ids)."""
    out = b"WPILOG" + struct.pack("<H", 0x0100) + struct.pack("<I", 0)
    out += _start(1, "NT:/SmartDashboard/RobotStatus/_pdh_voltage", "double")
    out += _start(2, "NT:/SmartDashboard/RobotStatus/_pdh_current", "double")
    out += _start(3, "DS:enabled", "boolean")
    out += _record(3, 100_000, b"\x01")
    for k in range(int(seconds * 10)):
        ts = 100_000 + k * 100_000
        out += _record(1, ts, struct.pack("<d", volts - 0.01 * (k % 20)))
        out += _record(2, ts, struct.pack("<d", 5.0 + (k % 7)))
    out += _record(3, 100_000 + int(seconds * 1_000_000), b"\x00")
    return out


@pytest.fixture
def settings(tmp_path):
    return Settings(data_dir=tmp_path / "data", allow_no_auth=True, frame_ancestors="'self' https://team.example.org")


@pytest.fixture
def app(settings):
    return create_app(settings, start_worker=False)


@pytest.fixture
def client(app):
    from fastapi.testclient import TestClient
    return TestClient(app)


@pytest.fixture
def wpilog_bytes():
    return make_wpilog()


@pytest.fixture
def uploaded(client, wpilog_bytes):
    """(client, meta) after uploading one log."""
    r = client.post("/api/upload", files={"log": ("WPILIB_20261011_204810_CAEL_E2.wpilog", wpilog_bytes, "application/octet-stream")})
    assert r.status_code == 200, r.text
    return client, r.json()
