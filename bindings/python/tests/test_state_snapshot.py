import ctypes
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from motorbridge.abi import Abi, CState, CStateMetadata
from motorbridge.core import Motor
from motorbridge.errors import CallError


def make_motor(lib):
    motor = Motor.__new__(Motor)
    motor._ptr = 1
    motor._abi = SimpleNamespace(lib=lib)
    return motor


def fill(state: CState, metadata: CStateMetadata) -> None:
    state.has_value = 1
    state.can_id = 2
    state.arbitration_id = 0xFD000002
    state.status_code = 0
    state.pos = 1.5
    state.vel = -0.25
    state.torq = 3.0
    state.t_mos = 30.0
    state.t_rotor = 31.0
    metadata.feedback_age_ns = 1_500_000_000
    metadata.feedback_sequence = (1 << 40) + 7


def test_snapshot_converts_age_and_preserves_the_full_counter():
    def read_snapshot(_handle, state, metadata):
        fill(
            ctypes.cast(state, ctypes.POINTER(CState))[0],
            ctypes.cast(metadata, ctypes.POINTER(CStateMetadata))[0],
        )
        return 0

    motor = make_motor(SimpleNamespace(motor_handle_get_state_snapshot=read_snapshot))
    snapshot = motor.get_state_snapshot()
    assert snapshot is not None
    assert snapshot.feedback_age_s == 1.5
    assert snapshot.feedback_sequence == (1 << 40) + 7
    assert snapshot.state.can_id == 2
    assert snapshot.state.arbitration_id == 0xFD000002
    assert snapshot.state.pos == 1.5
    assert snapshot.state.vel == -0.25
    assert snapshot.state.torq == 3.0
    assert snapshot.state.t_mos == 30.0
    assert snapshot.state.t_rotor == 31.0


def test_snapshot_returns_none_before_the_first_state_update():
    def read_snapshot(_handle, _state, _metadata):
        return 0

    motor = make_motor(SimpleNamespace(motor_handle_get_state_snapshot=read_snapshot))
    assert motor.get_state_snapshot() is None


def test_snapshot_raises_on_native_error(monkeypatch):
    monkeypatch.setattr("motorbridge.core._err_text", lambda: "motor is closed")
    motor = make_motor(SimpleNamespace(motor_handle_get_state_snapshot=lambda *_: -1))
    with pytest.raises(CallError, match="motor is closed"):
        motor.get_state_snapshot()


def test_snapshot_binding_uses_uint64_metadata():
    abi = Abi.__new__(Abi)
    abi.lib = Mock()
    abi._bind()
    snapshot = abi.lib.motor_handle_get_state_snapshot
    assert snapshot.argtypes == [
        ctypes.c_void_p,
        ctypes.POINTER(CState),
        ctypes.POINTER(CStateMetadata),
    ]
    assert snapshot.restype is ctypes.c_int32
    for field_type in (CStateMetadata.feedback_age_ns, CStateMetadata.feedback_sequence):
        assert field_type.size == ctypes.sizeof(ctypes.c_uint64)
