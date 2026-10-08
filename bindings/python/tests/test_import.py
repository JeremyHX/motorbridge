from motorbridge import Controller, Mode, MotorState, MotorStateSnapshot


def test_import_symbols() -> None:
    assert Controller is not None
    assert Mode.MIT.value == 1
    assert MotorState is not None
    assert MotorStateSnapshot is not None
