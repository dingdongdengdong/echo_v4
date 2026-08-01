import argparse

import pytest

from lerobot_robot_roboparty.can_probe import decode_dm4340p_state, parse_motor_ids


def test_parse_motor_ids_accepts_decimal_and_hex() -> None:
    assert parse_motor_ids("19, 0x14,21") == [19, 20, 21]


@pytest.mark.parametrize("value", ["", "0", "0x800"])
def test_parse_motor_ids_rejects_invalid_values(value: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        parse_motor_ids(value)


def test_decode_dm4340p_feedback() -> None:
    state = decode_dm4340p_state(bytes.fromhex("019c877ff7f81c1a"))

    assert state["status"] == "disabled"
    assert state["motor_id_low_nibble"] == 1
    assert state["position_rad"] == pytest.approx(2.786107, abs=1e-6)
    assert state["velocity_rad_s"] == pytest.approx(-0.004884, abs=1e-6)
    assert state["torque_nm"] == pytest.approx(-0.102564, abs=1e-6)
    assert state["mos_temperature_c"] == 28
    assert state["rotor_temperature_c"] == 26
