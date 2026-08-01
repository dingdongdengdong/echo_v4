from __future__ import annotations

import argparse
import sys
import time

CAN_PARAM_ID = 0x7FF
CAN_CMD_REFRESH = 0xCC
DM4340P_POSITION_LIMIT_RAD = 12.5
DM4340P_VELOCITY_LIMIT_RAD_S = 20.0
DM4340P_TORQUE_LIMIT_NM = 28.0

DM_STATUS = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "over_voltage",
    0x9: "under_voltage",
    0xA: "over_current",
    0xB: "mos_over_temperature",
    0xC: "coil_over_temperature",
    0xD: "lost_connection",
    0xE: "overload",
}


def parse_motor_ids(value: str) -> list[int]:
    ids = [int(item.strip(), 0) for item in value.split(",") if item.strip()]
    if not ids or any(motor_id < 1 or motor_id > 0x7FF for motor_id in ids):
        raise argparse.ArgumentTypeError("motor IDs must be comma-separated values in 1..0x7ff")
    return ids


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value / ((1 << bits) - 1) * (maximum - minimum) + minimum


def decode_dm4340p_state(data: bytes) -> dict[str, float | int | str]:
    if len(data) != 8:
        raise ValueError(f"expected an 8-byte Damiao feedback frame, received {len(data)} bytes")
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    status_code = data[0] >> 4
    return {
        "status_code": status_code,
        "status": DM_STATUS.get(status_code, f"unknown_0x{status_code:X}"),
        "motor_id_low_nibble": data[0] & 0x0F,
        "position_rad": uint_to_float(
            position_raw,
            -DM4340P_POSITION_LIMIT_RAD,
            DM4340P_POSITION_LIMIT_RAD,
            16,
        ),
        "velocity_rad_s": uint_to_float(
            velocity_raw,
            -DM4340P_VELOCITY_LIMIT_RAD_S,
            DM4340P_VELOCITY_LIMIT_RAD_S,
            12,
        ),
        "torque_nm": uint_to_float(
            torque_raw,
            -DM4340P_TORQUE_LIMIT_NM,
            DM4340P_TORQUE_LIMIT_NM,
            12,
        ),
        "mos_temperature_c": data[6],
        "rotor_temperature_c": data[7],
    }


def read_dm4340p_state(
    bus,
    motor_id: int,
    timeout_s: float,
    feedback_id: int | None = None,
) -> tuple[int, bytes, dict[str, float | int | str]] | None:
    import can

    expected_feedback_id = feedback_id if feedback_id is not None else motor_id + 0x10
    request = can.Message(
        arbitration_id=CAN_PARAM_ID,
        data=[motor_id & 0xFF, motor_id >> 8, CAN_CMD_REFRESH, 0, 0, 0, 0, 0],
        is_extended_id=False,
    )
    bus.send(request)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        reply = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if reply is None:
            return None
        if reply.arbitration_id != expected_feedback_id:
            continue
        data = bytes(reply.data)
        return reply.arbitration_id, data, decode_dm4340p_state(data)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Feedback-only Atom01/CANable probe; never enables torque or sends motion commands"
    )
    parser.add_argument("--port", required=True, help="CANable SLCAN serial port on the Mac")
    parser.add_argument("--motor-ids", type=parse_motor_ids, default=parse_motor_ids("19,20,21,22,23"))
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=float, default=0.15, help="Reply wait per motor, seconds")
    args = parser.parse_args()

    try:
        import can
    except ImportError:
        print("ERROR install the hardware extra: uv sync --extra hardware", file=sys.stderr)
        return 2

    print("SAFETY feedback-only mode: no enable, zero, disable, or position frame is sent")
    print(f"OPEN {args.port} as SLCAN at {args.bitrate} bit/s")
    try:
        bus = can.Bus(interface="slcan", channel=args.port, bitrate=args.bitrate)
    except Exception as exc:
        print(f"ERROR could not open CANable: {exc}", file=sys.stderr)
        return 2

    replies: list[tuple[int, int, str]] = []
    try:
        while bus.recv(timeout=0.01):
            pass
        for motor_id in args.motor_ids:
            result = read_dm4340p_state(bus, motor_id, args.timeout)
            if result is None:
                continue
            reply_id, data, state = result
            replies.append((motor_id, reply_id, data.hex()))
            print(
                f"HIT command=0x{motor_id:02X} feedback=0x{reply_id:02X} "
                f"state={state['status']} position={state['position_rad']:+.4f}rad "
                f"velocity={state['velocity_rad_s']:+.4f}rad/s "
                f"torque={state['torque_nm']:+.3f}Nm "
                f"temperature={state['mos_temperature_c']}/{state['rotor_temperature_c']}C"
            )
    except Exception as exc:
        print(f"ERROR CAN probe failed: {exc}", file=sys.stderr)
        return 2
    finally:
        bus.shutdown()

    if not replies:
        print("WAIT CANable opened, but no motor feedback arrived")
        return 1
    print(f"PASS received {len(replies)} CAN feedback frame(s)")
    return 0
