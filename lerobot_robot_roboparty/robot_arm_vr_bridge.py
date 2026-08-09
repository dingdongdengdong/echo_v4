from __future__ import annotations

import argparse
import sys
import threading
import time
from collections.abc import Callable
from contextlib import suppress
from pathlib import Path
from typing import Any

import numpy as np

from .amazing_hand import HAND_SERVO_IDS, AmazingHandBus, FullGraspMapper
from .can_probe import (
    DM4340P_POSITION_LIMIT_RAD,
    DM4340P_TORQUE_LIMIT_NM,
    DM4340P_VELOCITY_LIMIT_RAD_S,
    read_dm4340p_state,
)
from .two_motor_calibration import calibrated_position_rad, load_calibration
from .two_motor_quest_teleop import (
    CAN_CMD_DISABLE,
    CAN_CMD_ENABLE,
    MIT_KD_RANGE,
    MIT_KP_RANGE,
    _float_to_uint,
    _send_simple_command,
    logical_to_raw_position,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_ARM_VR_ROOT = REPO_ROOT / "third_party" / "robot_arm_vr"
DEFAULT_CONFIG = ROBOT_ARM_VR_ROOT / "config" / "superarm_j1_j2_jetson.json"
DEFAULT_CALIBRATION = REPO_ROOT / "config" / "right_arm_two_motor.json"
DEFAULT_RATE_HZ = 20.0
DEFAULT_MOTOR_RATE_HZ = 50.0
DEFAULT_VELOCITY_SCALE = 0.16  # ≈ 0.03 rad/frame × 20 Hz ÷ 3.77 rad/s
# Hardware-tuned on the shortened J1/J2-only arm.  J2 needs more stiffness and
# damping because it carries the arm against gravity; copying one gain to both
# axes produced either J2 droop or an unnecessarily stiff J1.
DEFAULT_KP = (120.0, 180.0)
DEFAULT_KD = (2.5, 4.0)
DEFAULT_CAN_PORT = (
    "/dev/serial/by-id/"
    "usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_"
    "2070388B3136-if00"
)
DEFAULT_HAND_PORT = (
    "/dev/serial/by-id/"
    "usb-1a86_USB_Single_Serial_5C63050237-if00"
)
DEFAULT_HAND_BAUDRATE = 1_000_000
DEFAULT_HAND_TIMEOUT_S = 0.5
DEFAULT_HAND_SPEED = 3
# hand_model.py의 기구학 한계는 +/-90도지만 끝점에 바로 붙이지 않고 2도 여유를 둔다.
HAND_SERVO_LIMIT_RAD = np.radians(88.0)
HAND_OPEN_LOGICAL_RAD = np.radians((-35.0, 35.0) * 4)
HAND_CLOSED_LOGICAL_RAD = np.radians((88.0, -88.0) * 4)
MAX_STALE_FEEDBACK_FRAMES = 4096


def logical_servo_to_bus_positions(servo_rad: list[float] | np.ndarray) -> dict[int, float]:
    """Convert robot_arm_vr logical servo angles to Rustypot bus angles.

    robot_arm_vr sends the AmazingHand demo's logical ID order.  The physical
    SCS0009 convention reverses every even-numbered servo, as does the verified
    AmazingHandControl ``angle_rad`` helper.
    """
    values = np.asarray(servo_rad, dtype=float)
    if values.shape != (len(HAND_SERVO_IDS),) or not np.isfinite(values).all():
        raise ValueError("AmazingHand servo command must contain eight finite angles")
    values = np.clip(values, -HAND_SERVO_LIMIT_RAD, HAND_SERVO_LIMIT_RAD)
    return {
        servo_id: float(-values[index] if servo_id % 2 == 0 else values[index])
        for index, servo_id in enumerate(HAND_SERVO_IDS)
    }


def grasp_to_logical_servo(grasp: float) -> np.ndarray:
    """Fallback 0..1 grasp command using the same servo-space interpolation as VR."""
    amount = float(np.clip(float(grasp), 0.0, 1.0))
    return HAND_OPEN_LOGICAL_RAD + amount * (
        HAND_CLOSED_LOGICAL_RAD - HAND_OPEN_LOGICAL_RAD
    )


class AmazingHandCommandSink:
    """Forward the proven LeRobot full-grasp motion from the Quest trigger.

    robot_arm_vr also sends an eight-servo visual-model pose, but that pose is
    not the gripping motion previously used on the real AmazingHand.  Prefer
    the scalar grasp command and restore FullGraspMapper's 0..110 degree motion.
    """

    def __init__(self, bus: AmazingHandBus) -> None:
        self.bus = bus
        self.mapper = FullGraspMapper()
        self._last_target: tuple[float, ...] | None = None

    def forward(self, command: Any | None) -> bool:
        if command is None:
            return False
        if command.grasp is not None:
            positions = self.mapper.targets_rad(float(command.grasp) * 100.0)
        elif command.servo is not None:
            logical = np.asarray(command.servo, dtype=float)
            positions = logical_servo_to_bus_positions(logical)
        else:
            return False
        target = tuple(positions[servo_id] for servo_id in HAND_SERVO_IDS)
        if self._last_target is not None and np.allclose(target, self._last_target, atol=1e-4):
            return False
        self.bus.write_positions(positions)
        self._last_target = target
        return True


def encode_mit_command(
    position_rad: float,
    velocity_rad_s: float,
    *,
    kp: float,
    kd: float,
    torque_nm: float = 0.0,
) -> bytes:
    """Encode one DM4340 MIT command including desired velocity."""
    position = _float_to_uint(
        position_rad,
        -DM4340P_POSITION_LIMIT_RAD,
        DM4340P_POSITION_LIMIT_RAD,
        16,
    )
    velocity = _float_to_uint(
        velocity_rad_s,
        -DM4340P_VELOCITY_LIMIT_RAD_S,
        DM4340P_VELOCITY_LIMIT_RAD_S,
        12,
    )
    kp_raw = _float_to_uint(kp, *MIT_KP_RANGE, 12)
    kd_raw = _float_to_uint(kd, *MIT_KD_RANGE, 12)
    torque = _float_to_uint(
        torque_nm,
        -DM4340P_TORQUE_LIMIT_NM,
        DM4340P_TORQUE_LIMIT_NM,
        12,
    )
    return bytes(
        [
            position >> 8,
            position & 0xFF,
            velocity >> 4,
            ((velocity & 0x0F) << 4) | (kp_raw >> 8),
            kp_raw & 0xFF,
            kd_raw >> 4,
            ((kd_raw & 0x0F) << 4) | (torque >> 8),
            torque & 0xFF,
        ]
    )


def raw_to_session_joint(raw: np.ndarray, origin: np.ndarray, signs: np.ndarray) -> np.ndarray:
    """Convert raw motor radians to session-relative joint radians."""
    return signs * (np.asarray(raw, dtype=float) - np.asarray(origin, dtype=float))


def session_joint_to_raw(q: np.ndarray, origin: np.ndarray, signs: np.ndarray) -> np.ndarray:
    """Convert session-relative joint radians to raw motor radians."""
    return np.asarray(origin, dtype=float) + signs * np.asarray(q, dtype=float)


def raw_to_calibrated_joint(
    raw: np.ndarray,
    motors: tuple[dict, dict],
    signs: np.ndarray,
) -> np.ndarray:
    """Convert raw motor radians to calibrated URDF joint radians."""
    logical = np.asarray(
        [calibrated_position_rad(float(value), motor) for value, motor in zip(raw, motors, strict=True)],
        dtype=float,
    )
    return np.asarray(signs, dtype=float) * logical


def calibrated_joint_to_raw(
    q: np.ndarray,
    motors: tuple[dict, dict],
    signs: np.ndarray,
) -> np.ndarray:
    """Convert calibrated URDF joint radians to raw motor radians."""
    logical = np.asarray(signs, dtype=float) * np.asarray(q, dtype=float)
    return np.asarray(
        [logical_to_raw_position(float(value), motor) for value, motor in zip(logical, motors, strict=True)],
        dtype=float,
    )


def calibrated_joint_limits(
    motors: tuple[dict, dict],
    signs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return calibration capture limits expressed in URDF joint coordinates."""
    lower = []
    upper = []
    for motor, sign in zip(motors, signs, strict=True):
        endpoints = np.asarray(
            [float(motor["range_min_rad"]), float(motor["range_max_rad"])],
            dtype=float,
        ) * float(sign)
        lower.append(float(endpoints.min()))
        upper.append(float(endpoints.max()))
    return np.asarray(lower), np.asarray(upper)


def select_calibration_motors(calibration: dict, motor_ids: tuple[int, int]) -> tuple[dict, dict]:
    """Select J1/J2 calibration records in command-ID order."""
    by_id = {int(motor["command_id"]): motor for motor in calibration["motors"]}
    missing = [motor_id for motor_id in motor_ids if motor_id not in by_id]
    if missing:
        raise ValueError(f"calibration is missing motor command IDs: {missing}")
    return tuple(by_id[motor_id] for motor_id in motor_ids)  # type: ignore[return-value]


class RawRelativeDMBackend:
    """Two DM4340 motors using saved calibration or a volatile session zero.

    When ``calibration`` is supplied, state and commands use calibrated LeRobot
    joint radians. Otherwise the current raw J1/J2 positions become joint [0, 0].
    """

    def __init__(
        self,
        can_port: str,
        *,
        motor_ids: tuple[int, int] = (1, 2),
        feedback_ids: tuple[int, int] = (0x11, 0x12),
        signs: tuple[float, float] = (1.0, 1.0),
        bitrate: int = 1_000_000,
        timeout_s: float = 0.05,
        kp: tuple[float, float] = DEFAULT_KP,
        kd: tuple[float, float] = DEFAULT_KD,
        max_velocity_rad_s: tuple[float, float] = (3.77, 3.77),
        calibration: dict | None = None,
        bus_factory: Callable[..., Any] | None = None,
    ) -> None:
        if len(set(motor_ids)) != 2 or len(set(feedback_ids)) != 2:
            raise ValueError("J1/J2 motor and feedback IDs must be unique")
        if any(sign not in (-1.0, 1.0) for sign in signs):
            raise ValueError("motor signs must be +1 or -1")
        if timeout_s <= 0 or bitrate <= 0:
            raise ValueError("timeout and bitrate must be positive")

        self.can_port = can_port
        self.motor_ids = tuple(int(value) for value in motor_ids)
        self.feedback_ids = tuple(int(value) for value in feedback_ids)
        self.signs = np.asarray(signs, dtype=float)
        self.bitrate = int(bitrate)
        self.timeout_s = float(timeout_s)
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.max_velocity = np.asarray(max_velocity_rad_s, dtype=float)
        self.calibration_motors = (
            None if calibration is None else select_calibration_motors(calibration, self.motor_ids)
        )
        self._bus_factory = bus_factory
        self._bus = None
        self._raw_origin: np.ndarray | None = None
        self._last_q_cmd: np.ndarray | None = None
        self._last_write_at: float | None = None
        self._last_states: list[dict[str, float | int | str]] = []
        self._enabled = False
        self._io_lock = threading.RLock()

    @property
    def n_joints(self) -> int:
        return 2

    @property
    def raw_origin(self) -> np.ndarray | None:
        return None if self._raw_origin is None else self._raw_origin.copy()

    @property
    def start_q(self) -> np.ndarray | None:
        """Joint position captured during connect, without another CAN transaction."""
        return None if self._last_q_cmd is None else self._last_q_cmd.copy()

    def connect(self) -> None:
        if self._bus is not None:
            return
        import can

        factory = self._bus_factory or can.Bus
        self._bus = factory(interface="slcan", channel=self.can_port, bitrate=self.bitrate)
        while self._bus.recv(timeout=0.01):
            pass
        _send_simple_command(self._bus, list(self.motor_ids), CAN_CMD_DISABLE)
        raw = self._read_raw()
        self._raw_origin = raw.copy()
        self._last_q_cmd = self._raw_to_joint(raw)
        self._last_write_at = time.monotonic()
        self._enabled = False

    def disconnect(self) -> None:
        if self._bus is None:
            return
        with suppress(Exception):
            self.disable()
        with suppress(Exception):
            self._bus.shutdown()
        self._bus = None

    def enable(self) -> None:
        with self._io_lock:
            self._require_connected()
            if not self._enabled:
                _send_simple_command(self._bus, list(self.motor_ids), CAN_CMD_ENABLE)
                self._enabled = True
                self._last_write_at = time.monotonic()

    def disable(self) -> None:
        with self._io_lock:
            self._require_connected()
            _send_simple_command(self._bus, list(self.motor_ids), CAN_CMD_DISABLE)
            self._enabled = False

    def read_positions(self) -> np.ndarray:
        raw = self._read_raw()
        if self._raw_origin is None:
            raise RuntimeError("raw session origin is not initialized")
        return self._raw_to_joint(raw)

    def read_velocities(self) -> np.ndarray:
        if len(self._last_states) != 2:
            self.read_positions()
        raw = np.asarray([float(state["velocity_rad_s"]) for state in self._last_states])
        return self.signs * raw

    def read_temperatures(self) -> np.ndarray:
        if len(self._last_states) != 2:
            self.read_positions()
        return np.asarray([float(state["mos_temperature_c"]) for state in self._last_states])

    def read_errors(self) -> list[int]:
        if len(self._last_states) != 2:
            self.read_positions()
        return [0 if state["status"] in ("disabled", "enabled") else int(state["status_code"])
                for state in self._last_states]

    def write_positions(self, q: np.ndarray, dq: np.ndarray | None = None) -> None:
        del dq  # Desired velocity must follow the safety-clamped command, not upstream intent.
        with self._io_lock:
            self._require_connected()
            if self._raw_origin is None:
                raise RuntimeError("raw session origin is not initialized")
            q = np.asarray(q, dtype=float)
            if q.shape != (2,) or not np.isfinite(q).all():
                raise ValueError("expected two finite J1/J2 targets")

            now = time.monotonic()
            previous = q if self._last_q_cmd is None else self._last_q_cmd
            elapsed = max(1e-3, now - self._last_write_at) if self._last_write_at is not None else 1.0 / 30.0
            desired_velocity = np.clip((q - previous) / elapsed, -self.max_velocity, self.max_velocity)
            if self.calibration_motors is not None:
                lower, upper = calibrated_joint_limits(self.calibration_motors, self.signs)
                q = np.clip(q, lower, upper)
                raw_targets = calibrated_joint_to_raw(q, self.calibration_motors, self.signs)
            else:
                raw_targets = session_joint_to_raw(q, self._raw_origin, self.signs)
            raw_velocities = self.signs * desired_velocity

            import can

            for motor_id, target, velocity, kp, kd in zip(
                self.motor_ids,
                raw_targets,
                raw_velocities,
                self.kp,
                self.kd,
                strict=True,
            ):
                self._bus.send(
                    can.Message(
                        arbitration_id=motor_id,
                        data=encode_mit_command(float(target), float(velocity), kp=float(kp), kd=float(kd)),
                        is_extended_id=False,
                    )
                )
                time.sleep(0.001)
            self._last_q_cmd = q.copy()
            self._last_write_at = now

    def _raw_to_joint(self, raw: np.ndarray) -> np.ndarray:
        if self.calibration_motors is not None:
            return raw_to_calibrated_joint(raw, self.calibration_motors, self.signs)
        if self._raw_origin is None:
            raise RuntimeError("raw session origin is not initialized")
        return raw_to_session_joint(raw, self._raw_origin, self.signs)

    def _read_raw(self) -> np.ndarray:
        with self._io_lock:
            self._require_connected()
            # An MIT position command already produces one feedback frame per motor.
            # read_dm4340p_state() then sends an explicit refresh request as well.  If
            # the command replies are left queued, each control cycle produces four
            # frames but consumes only two and the state estimate falls seconds behind
            # the real arm.  Discard those asynchronous command replies before asking
            # for a fresh, ID-matched sample from each motor.
            for _ in range(MAX_STALE_FEEDBACK_FRAMES):
                if self._bus.recv(timeout=0.0) is None:
                    break
            states = []
            for motor_id, feedback_id in zip(self.motor_ids, self.feedback_ids, strict=True):
                result = read_dm4340p_state(self._bus, motor_id, self.timeout_s, feedback_id)
                if result is None:
                    raise ConnectionError(f"lost raw feedback from motor {motor_id}")
                states.append(result[2])
            self._last_states = states
            return np.asarray([float(state["position_rad"]) for state in states])

    def _require_connected(self) -> None:
        if self._bus is None:
            raise RuntimeError("CAN backend is not connected")


def _load_robot_arm_vr():
    source = ROBOT_ARM_VR_ROOT / "src"
    if not source.is_dir():
        raise FileNotFoundError(
            f"missing {ROBOT_ARM_VR_ROOT}; clone https://github.com/beautifulmelon/robot_arm_vr first"
        )
    sys.path.insert(0, str(source))
    from rpo_teleop.arm_config import ArmConfig
    from rpo_teleop.jetson_sim import FakeJetson
    from rpo_teleop.profiles import ports

    return ArmConfig, FakeJetson, ports


def main() -> int:
    parser = argparse.ArgumentParser(
        description="robot_arm_vr UDP bridge to calibrated J1/J2 DM4340 motors"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--can-port", default=DEFAULT_CAN_PORT)
    parser.add_argument("--hand-port", default=DEFAULT_HAND_PORT)
    parser.add_argument("--hand-baudrate", type=int, default=DEFAULT_HAND_BAUDRATE)
    parser.add_argument("--hand-timeout", type=float, default=DEFAULT_HAND_TIMEOUT_S)
    parser.add_argument("--hand-speed", type=int, default=DEFAULT_HAND_SPEED)
    parser.add_argument("--no-hand", action="store_true",
                        help="do not connect or command the AmazingHand")
    parser.add_argument("--host", default="", help="UDP bind address")
    parser.add_argument("--profile", choices=("jetson", "isaac", "test"), default="jetson")
    parser.add_argument("--velocity-scale", type=float, default=DEFAULT_VELOCITY_SCALE)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--motor-rate", type=float, default=DEFAULT_MOTOR_RATE_HZ)
    parser.add_argument("--motor1-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--motor2-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--kp", type=float, nargs=2, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, nargs=2, default=DEFAULT_KD)
    args = parser.parse_args()
    if args.velocity_scale <= 0 or args.rate <= 0 or args.motor_rate <= 0:
        parser.error("velocity scale, state rate, and motor rate must be positive")
    if args.hand_baudrate <= 0 or args.hand_timeout <= 0 or not 1 <= args.hand_speed <= 6:
        parser.error("hand baudrate/timeout must be positive and hand speed must be 1..6")
    ArmConfig, FakeJetson, ports = _load_robot_arm_vr()
    cfg = ArmConfig.load(args.config)
    if cfg.dof < 2:
        parser.error("robot_arm_vr config must contain at least J1/J2")
    pf = ports(args.profile)
    calibration = load_calibration(args.calibration)
    calibration_motors = select_calibration_motors(calibration, (1, 2))
    signs = np.asarray((args.motor1_sign, args.motor2_sign), dtype=float)
    calibration_lower, calibration_upper = calibrated_joint_limits(calibration_motors, signs)
    effective_lower = np.asarray(cfg.lower, dtype=float).copy()
    effective_upper = np.asarray(cfg.upper, dtype=float).copy()
    effective_lower[:2] = np.maximum(effective_lower[:2], calibration_lower)
    effective_upper[:2] = np.minimum(effective_upper[:2], calibration_upper)
    if np.any(effective_lower[:2] >= effective_upper[:2]):
        parser.error("calibration range does not overlap the J1/J2 URDF limits")
    backend = RawRelativeDMBackend(
        args.can_port,
        signs=tuple(signs),
        kp=tuple(args.kp),
        kd=tuple(args.kd),
        max_velocity_rad_s=tuple(np.asarray(cfg.velocity[:2], dtype=float)),
        calibration=calibration,
    )
    jet = FakeJetson(
        lower=effective_lower,
        upper=effective_upper,
        max_velocity=cfg.velocity,
        n_joints=cfg.dof,
        n_motors=2,
        dt=cfg.control_dt,
        rate_hz=args.rate,
        motor_hz=args.motor_rate,
        velocity_scale=args.velocity_scale,
        cmd_port=pf.cmd,
        state_port=pf.state,
        beacon_port=pf.beacon,
        robot=Path(cfg.urdf_path).name,
        host=args.host,
    )
    jet.motors = backend
    hand = None if args.no_hand else AmazingHandBus(
        args.hand_port,
        args.hand_baudrate,
        args.hand_timeout,
        args.hand_speed,
    )
    hand_sink = None if hand is None else AmazingHandCommandSink(hand)

    try:
        if hand is not None:
            hand.connect()
        jet.start()
        start_q = backend.start_q
        if start_q is None:
            raise RuntimeError("J1/J2 start position was not captured")
        print("=" * 74)
        print("robot_arm_vr -> calibrated J1/J2 Jetson bridge")
        print(f"calibration: {args.calibration.resolve()}")
        print(f"calibrated start J1/J2: {np.round(start_q, 4).tolist()} rad")
        print(
            "effective limits J1/J2: "
            f"{np.round(effective_lower[:2], 4).tolist()} .. "
            f"{np.round(effective_upper[:2], 4).tolist()} rad"
        )
        print(f"motor signs: {backend.signs.tolist()}")
        print(f"motor profile (hardware-tuned): kp={backend.kp.tolist()} kd={backend.kd.tolist()}")
        print(f"UDP command/state/beacon: {pf.cmd}/{pf.state}/{pf.beacon}")
        print(
            f"velocity scale: {args.velocity_scale:.2f}; motor interpolation: "
            f"{args.motor_rate:.0f} Hz; only motor IDs 1 and 2 are used"
        )
        if hand is None:
            print("AmazingHand: disabled (--no-hand)")
        else:
            print(
                f"AmazingHand: {args.hand_port} @ {args.hand_baudrate}; "
                f"Trigger controls IDs 1..8 independently of arm HOLD/RUN"
            )
        print("Waiting for robot_arm_vr Mac teleop; Ctrl+C stops and disables J1/J2")
        print("=" * 74, flush=True)
        previous_status = None
        last_stats_log_s = 0.0
        hand_failed = False
        while True:
            time.sleep(0.02)
            if hand_sink is not None and not hand_failed:
                try:
                    hand_sink.forward(jet.last_cmd)
                except Exception as exc:
                    hand_failed = True
                    with suppress(Exception):
                        hand.disconnect()
                    print(
                        f"AmazingHand disabled after command failure: {type(exc).__name__}: {exc}",
                        flush=True,
                    )
            status = (jet.state, jet._link, jet.stats["sessions"])
            now = time.monotonic()
            if status != previous_status or now - last_stats_log_s >= 5.0:
                print(
                    f"state={status[0]} link={status[1]} rx={jet.stats['rx']} "
                    f"tx={jet.stats['tx']} sessions={status[2]}",
                    flush=True,
                )
                previous_status = status
                last_stats_log_s = now
    except KeyboardInterrupt:
        return 0
    finally:
        with suppress(Exception):
            jet.stop()
        if hand is not None:
            with suppress(Exception):
                hand.disconnect()
        backend.disconnect()
        print("STOPPED J1/J2 and AmazingHand torque=disabled", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
