"""Commissioning tool for the arm_v2 STS3215 parallel gripper.

``parallel_gripper.py`` maps ``Command.grasp`` onto the sweep angle used by the
mechanical designer's ``gripper_map.py``.  Turning that sweep angle into a bus
angle needs two numbers that only the built gripper can tell us: where the
servo reads when the sweep says zero, and which way it turns.  Guessing either
one drives the jaws the wrong way, so the bridge refuses to start without them
and this tool measures them.

Subcommands, in the order they are used:

    scan     which servo IDs answer on the bus
    read     watch the servo angle while moving the jaws by hand
    jog      nudge the servo a relative amount to free jaws that are stuck
    measure  capture the two endpoints by hand and solve for zero and sign
    autozero same result without hands, by driving to the open stop
    verify   drive the sweep with the solved numbers and check it tracks

``jog``, ``autozero`` and ``verify`` energise the servo.  Everything else reads, or
releases torque so the jaws can be moved by hand.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Sequence
from contextlib import suppress
from pathlib import Path

import numpy as np

from .parallel_gripper import (
    DEFAULT_GRIPPER_BAUDRATE,
    DEFAULT_GRIPPER_ID,
    DEFAULT_GRIPPER_MAP,
    BUS_RETRIES,
    BUS_RETRY_DELAY_S,
    ParallelGripperBus,
    grasp_to_servo_deg,
    load_gripper_map,
    make_rustypot_controller,
)

# Scanning the whole protocol range is slow and pointless; a gripper servo is
# hand-assigned a low ID.
SCAN_ID_MAX = 30
COMMISSIONING_TIMEOUT_S = 1.0


def _scalar(value) -> float:
    """Unwrap a rustypot reading.

    The controllers answer with a one-element list rather than a number.
    Calling float() on that raises TypeError, which reads exactly like a servo
    that did not answer, so unwrap before converting.
    """
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) != 1:
            raise ConnectionError(f"expected one reading, received {len(value)}")
        value = value[0]
    return float(value)


def _read_deg(controller, servo_id: int) -> float:
    """Present position in degrees, retrying the transient serial timeouts."""
    last: Exception | None = None
    for attempt in range(BUS_RETRIES):
        try:
            return math.degrees(_scalar(controller.read_present_position(servo_id)))
        except (RuntimeError, OSError) as exc:
            last = exc
            if attempt + 1 < BUS_RETRIES:
                time.sleep(BUS_RETRY_DELAY_S)
    raise ConnectionError(f"read failed {BUS_RETRIES} times: {last}") from last


def _controller(args) -> object:
    return make_rustypot_controller(args.port, args.baudrate, args.timeout)


def scan(args) -> int:
    controller = _controller(args)
    print(f"scanning IDs 1..{SCAN_ID_MAX} on {args.port} @ {args.baudrate}")
    found: list[tuple[int, float]] = []
    for servo_id in range(1, SCAN_ID_MAX + 1):
        try:
            position = _scalar(controller.read_present_position(servo_id))
        except (ConnectionError, TypeError) as exc:
            # A malformed reading is a bug here, not a silent servo; saying so
            # beats reporting an empty bus.
            print(f"  id={servo_id} answered but could not be read: {exc}",
                  file=sys.stderr)
            continue
        except Exception:
            continue
        found.append((servo_id, position))
        print(f"  HIT id={servo_id}  position={math.degrees(position):+8.2f} deg")
    if not found:
        print(
            "no servo answered.\n"
            "  - is the servo powered? the bus adapter alone does not power it\n"
            "  - is --baudrate right? the AmazingHand bus runs at 1000000\n"
            f"  - is {args.port} the servo adapter and not the CAN adapter?",
            file=sys.stderr,
        )
        return 1
    print(f"PASS {len(found)} servo(s) answered")
    return 0


def read(args) -> int:
    """Release torque and print the live angle so the jaws can be moved by hand."""
    controller = _controller(args)
    controller.write_torque_enable(args.id, False)
    print(f"torque released on id={args.id}; move the jaws by hand.  Ctrl+C to stop")
    try:
        while True:
            position = math.radians(_read_deg(controller, args.id))
            print(f"\r  bus {math.degrees(position):+9.3f} deg   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
        return 0


def solve_zero_and_sign(
    open_bus_deg: float,
    closed_bus_deg: float,
    open_sweep_deg: float,
    closed_sweep_deg: float,
) -> tuple[float, float, float]:
    """Fit ``bus = zero + sign * sweep`` through the two captured endpoints.

    Returns ``(zero_deg, sign, slope)``.  The slope should be very close to
    +-1: the sweep is already in servo degrees, so anything else means the two
    endpoints were captured at the wrong jaw positions or the servo is geared.
    """
    sweep_span = closed_sweep_deg - open_sweep_deg
    if abs(sweep_span) < 1e-6:
        raise ValueError("the two endpoints must be different sweep angles")
    slope = (closed_bus_deg - open_bus_deg) / sweep_span
    if slope == 0.0 or not math.isfinite(slope):
        raise ValueError(
            "the servo did not move between the two endpoints; capture them again"
        )
    sign = 1.0 if slope > 0 else -1.0
    zero_deg = open_bus_deg - sign * open_sweep_deg
    return zero_deg, sign, slope


def _capture(controller, servo_id: int, samples: int, interval: float) -> float:
    values = []
    for _ in range(samples):
        values.append(math.radians(_read_deg(controller, servo_id)))
        time.sleep(interval)
    array = np.degrees(np.asarray(values, dtype=float))
    spread = float(array.max() - array.min())
    if spread > 1.0:
        raise ValueError(
            f"the servo moved {spread:.2f} deg while capturing; hold it still"
        )
    return float(array.mean())


def measure(args) -> int:
    gripper_map = load_gripper_map(args.gripper_map)
    open_sweep = grasp_to_servo_deg(0.0, gripper_map)
    closed_sweep = grasp_to_servo_deg(1.0, gripper_map)

    controller = _controller(args)
    controller.write_torque_enable(args.id, False)
    print("=" * 74)
    print("  arm_v2 gripper zero and sign")
    print("=" * 74)
    print(f"  gripper map : {Path(args.gripper_map).resolve()}")
    print(f"  sweep       : fully open {open_sweep:.1f} deg, "
          f"fully closed {closed_sweep:.1f} deg")
    print("  torque is released; the jaws move by hand")
    print()

    input("  1) open the jaws FULLY, hold them there, then press ENTER ")
    open_bus = _capture(controller, args.id, args.samples, args.sample_interval)
    print(f"     captured {open_bus:+.3f} deg")

    input("  2) close the jaws FULLY, hold them there, then press ENTER ")
    closed_bus = _capture(controller, args.id, args.samples, args.sample_interval)
    print(f"     captured {closed_bus:+.3f} deg")

    zero_deg, sign, slope = solve_zero_and_sign(
        open_bus, closed_bus, open_sweep, closed_sweep
    )
    print()
    print(f"  --gripper-zero-deg {zero_deg:.3f}")
    print(f"  --gripper-sign {sign:+.0f}")
    print(f"  (fitted slope {slope:+.4f}; should be near {sign:+.0f})")
    if abs(abs(slope) - 1.0) > 0.15:
        print()
        print(
            "  WARNING the slope is far from +-1. The sweep is already in servo\n"
            "  degrees, so this usually means one endpoint was not at the real\n"
            "  mechanical limit. Capture again before using these numbers.",
            file=sys.stderr,
        )
        return 1
    print()
    print("  check them with:")
    print(f"    roboparty-calibrate-gripper verify --port {args.port} "
          f"--id {args.id} --zero-deg {zero_deg:.3f} --sign {sign:+.0f}")
    return 0


def jog(args) -> int:
    """Nudge the servo a relative amount so a stuck gripper can be freed by hand.

    Zero and sign are what ``measure`` produces, so this cannot use them: it
    works purely in bus degrees relative to wherever the servo is now, which
    means it can never jump somewhere unexpected. Travel per call is capped and
    the move is ramped, so a typo moves the jaws a little rather than a lot.

    Torque is released when the move finishes, because the reason to nudge the
    jaws is to then move them by hand.
    """
    controller = _controller(args)
    start = _read_deg(controller, args.id)
    target = start + args.by

    print(f"  now {start:+.2f} deg  ->  {target:+.2f} deg   ({args.by:+.1f} deg)")
    if args.by == 0.0:
        print("  nothing to do")
        return 0

    controller.write_torque_enable(args.id, True)
    try:
        steps = max(1, int(abs(args.by) / args.step))
        for value in np.linspace(start, target, steps + 1)[1:]:
            controller.write_goal_position(args.id, math.radians(float(value)))
            time.sleep(args.settle)
        time.sleep(0.2)
        actual = _read_deg(controller, args.id)
        moved = actual - start
        print(f"  reached {actual:+.2f} deg  (moved {moved:+.2f} of {args.by:+.1f})")
        if abs(moved) < abs(args.by) * 0.5:
            print(
                "  the servo did not reach the target. It is probably against a\n"
                "  mechanical stop or the cover; do not keep pushing the same way.",
                file=sys.stderr,
            )
            return 1
    finally:
        if not args.hold:
            controller.write_torque_enable(args.id, False)
            print("  torque released; the jaws move by hand")
        else:
            print("  torque HELD; release it with --by 0 or by power cycling")
    return 0


def find_stop(controller, servo_id: int, direction: float, *,
              step: float, settle: float, travel: float,
              stall_ratio: float, stalls: int) -> tuple[float, float]:
    """Creep until the servo stops following, and report where it stopped.

    Returns ``(stop_deg, travelled_deg)``. Detection is by tracking error
    rather than by a current reading, because a jaw touching its stop still
    draws little current until the servo winds up against it.
    """
    position = _read_deg(controller, servo_id)
    start = position
    stalled = 0
    while abs(position - start) < travel:
        commanded = position + direction * step
        controller.write_goal_position(servo_id, math.radians(commanded))
        time.sleep(settle)
        moved_to = _read_deg(controller, servo_id)
        achieved = abs(moved_to - position)
        position = moved_to
        if achieved < step * stall_ratio:
            stalled += 1
            if stalled >= stalls:
                return position, position - start
        else:
            stalled = 0
    raise RuntimeError(
        f"travelled {travel:.0f} deg without finding a stop; the jaws are free "
        "to keep turning, so this is the wrong servo or the wrong direction"
    )


def autozero(args) -> int:
    """Find the open stop by driving to it, and solve zero and sign from it.

    Doing this without hands needs one endpoint the gripper can find on its
    own. The open stop is a hard mechanical limit, so it is that one; the
    closed end is not usable because the jaws meet before the linkage does and
    the fit is not even monotonic below about 27 deg.

    Sign then comes from which way the jaws opened, which the caller states,
    and zero follows from ``bus = zero + sign * sweep`` at the open stop.
    """
    gripper_map = load_gripper_map(args.gripper_map)
    open_sweep = grasp_to_servo_deg(0.0, gripper_map)
    closed_sweep = grasp_to_servo_deg(1.0, gripper_map)

    controller = _controller(args)
    restore_limit = None
    if args.torque_limit is not None:
        with suppress(Exception):
            restore_limit = _scalar(controller.read_torque_limit(args.id))
            controller.write_torque_limit(args.id, args.torque_limit)

    print("=" * 74)
    print("  finding the open stop")
    print("=" * 74)
    print(f"  opening direction : {args.open_sign:+.0f} on the bus")
    print(f"  torque limit      : "
          f"{'unchanged' if restore_limit is None else f'{args.torque_limit} (was {restore_limit:g})'}")
    print("  NOTHING BETWEEN THE JAWS, HANDS CLEAR")
    print()

    try:
        controller.write_torque_enable(args.id, True)
        stop_deg, travelled = find_stop(
            controller, args.id, args.open_sign,
            step=args.step, settle=args.settle, travel=args.travel,
            stall_ratio=args.stall_ratio, stalls=args.stalls,
        )
        print(f"  open stop at {stop_deg:+.2f} deg after {travelled:+.2f} deg of travel")
        # Sit just inside the stop instead of leaning on it.
        controller.write_goal_position(
            args.id, math.radians(stop_deg - args.open_sign * args.backoff))
        time.sleep(0.3)
    except Exception as exc:
        print(f"  FAILED {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        with suppress(Exception):
            controller.write_torque_enable(args.id, False)
        if restore_limit is not None:
            with suppress(Exception):
                controller.write_torque_limit(args.id, int(restore_limit))

    sign = float(args.open_sign)
    zero_deg = stop_deg - sign * open_sweep
    print()
    print(f"  --gripper-zero-deg {zero_deg:.3f}")
    print(f"  --gripper-sign {sign:+.0f}")
    print()
    print(f"  predicted closed (grasp 1, sweep {closed_sweep:.1f} deg) at "
          f"{zero_deg + sign * closed_sweep:+.2f} deg on the bus")
    print("  check it with:")
    print(f"    roboparty-calibrate-gripper --port {args.port} --id {args.id} "
          f"verify --zero-deg {zero_deg:.3f} --sign {sign:+.0f}")
    return 0


def verify(args) -> int:
    """Drive the sweep with the solved numbers and report tracking error.

    This is the only subcommand that energises the servo.
    """
    gripper_map = load_gripper_map(args.gripper_map)
    bus = ParallelGripperBus(args.port, args.baudrate, args.timeout, args.id)
    bus.connect()
    print("=" * 74)
    print(f"  driving the sweep with zero {args.zero_deg:+.3f} deg "
          f"sign {args.sign:+.0f}")
    print("  NOTHING BETWEEN THE JAWS, HANDS CLEAR")
    print("=" * 74)

    def bus_rad(sweep_deg: float) -> float:
        return math.radians(args.zero_deg + args.sign * sweep_deg)

    grasps = list(np.linspace(0.0, 1.0, args.steps))
    worst = 0.0
    try:
        # Walk from wherever the jaws are to the first target before timing
        # anything, so the run does not open with a jump across the sweep.
        first = grasp_to_servo_deg(grasps[0], gripper_map)
        start = (math.degrees(bus.read_position_rad()) - args.zero_deg) / args.sign
        for step in np.linspace(start, first, max(2, int(abs(first - start) / 5) + 1)):
            bus.write_position_rad(bus_rad(float(step)))
            time.sleep(0.05)

        for grasp in grasps + grasps[::-1]:
            target_sweep = grasp_to_servo_deg(grasp, gripper_map)
            bus.write_position_rad(bus_rad(target_sweep))
            time.sleep(args.settle)
            # A reading taken while the jaws are still travelling looks exactly
            # like a mis-calibration, so give a large error more settling time
            # and look again before believing it.
            for look in range(args.rereads + 1):
                actual = math.degrees(bus.read_position_rad())
                actual_sweep = (actual - args.zero_deg) / args.sign
                error = actual_sweep - target_sweep
                if abs(error) <= args.max_error_deg or look == args.rereads:
                    break
                time.sleep(args.settle)
            worst = max(worst, abs(error))
            print(f"  grasp {grasp:4.2f}  sweep target {target_sweep:6.1f}  "
                  f"actual {actual_sweep:6.1f}  error {error:+5.1f} deg")
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        bus.disconnect()

    print()
    print(f"  worst tracking error {worst:.1f} deg")
    if worst > args.max_error_deg:
        print(
            f"  FAIL tracking error exceeds {args.max_error_deg:.1f} deg. The jaws\n"
            "  are binding, the sign is inverted, or the zero is off.",
            file=sys.stderr,
        )
        return 1
    print("  PASS the gripper follows the sweep")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure the arm_v2 gripper servo zero and rotation direction"
    )
    parser.add_argument("--port", required=True,
                        help="servo bus adapter, e.g. /dev/serial/by-id/...")
    parser.add_argument("--id", type=int, default=DEFAULT_GRIPPER_ID)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_GRIPPER_BAUDRATE)
    # The bridge default is deliberately tight so a stalled bus cannot block the
    # control loop. Commissioning has no such deadline and a 0.5 s read does
    # time out on this bus often enough to corrupt a measurement, so wait longer.
    parser.add_argument("--timeout", type=float, default=COMMISSIONING_TIMEOUT_S)
    parser.add_argument("--gripper-map", type=Path, default=DEFAULT_GRIPPER_MAP)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("scan", help="list servo IDs answering on the bus")
    sub.add_parser("read", help="release torque and print the live angle")

    p_measure = sub.add_parser("measure", help="capture both endpoints and solve")
    p_measure.add_argument("--samples", type=int, default=15)
    p_measure.add_argument("--sample-interval", type=float, default=0.03)

    p_jog = sub.add_parser(
        "jog", help="nudge the servo a relative amount, then release torque")
    p_jog.add_argument("--by", type=float, required=True,
                       help="degrees to move from the current position, + or -")
    p_jog.add_argument("--step", type=float, default=2.0,
                       help="ramp granularity in degrees")
    p_jog.add_argument("--settle", type=float, default=0.08)
    p_jog.add_argument("--limit", type=float, default=30.0,
                       help="largest move allowed in one call")
    p_jog.add_argument("--hold", action="store_true",
                       help="keep torque on after the move")

    p_auto = sub.add_parser(
        "autozero", help="drive to the open stop and solve zero and sign, no hands")
    p_auto.add_argument("--open-sign", type=float, choices=(-1.0, 1.0), required=True,
                        help="bus direction that OPENS the jaws; find it with jog")
    p_auto.add_argument("--step", type=float, default=1.5)
    p_auto.add_argument("--settle", type=float, default=0.12)
    p_auto.add_argument("--travel", type=float, default=150.0,
                        help="give up after this much travel with no stop")
    p_auto.add_argument("--stall-ratio", type=float, default=0.35,
                        help="a step achieving less than this fraction counts as stalled")
    p_auto.add_argument("--stalls", type=int, default=3,
                        help="consecutive stalled steps that mean a stop")
    p_auto.add_argument("--backoff", type=float, default=3.0,
                        help="degrees to come off the stop when finished")
    p_auto.add_argument("--torque-limit", type=int, default=300,
                        help="limit while searching, restored afterwards")

    p_verify = sub.add_parser("verify", help="drive the sweep and check tracking")
    p_verify.add_argument("--zero-deg", type=float, required=True)
    p_verify.add_argument("--sign", type=float, choices=(-1.0, 1.0), required=True)
    p_verify.add_argument("--steps", type=int, default=6)
    p_verify.add_argument("--settle", type=float, default=0.9)
    p_verify.add_argument("--max-error-deg", type=float, default=8.0)
    p_verify.add_argument("--rereads", type=int, default=2,
                          help="extra settle-and-look attempts before failing a step")

    args = parser.parse_args()
    if args.baudrate <= 0 or args.timeout <= 0:
        parser.error("baudrate and timeout must be positive")
    if not 0 <= args.id <= 253:
        parser.error("servo ID must be in 0..253")

    if args.command == "jog":
        if args.step <= 0 or args.settle <= 0 or args.limit <= 0:
            parser.error("--step, --settle and --limit must be positive")
        if abs(args.by) > args.limit:
            parser.error(
                f"--by {args.by:+.1f} exceeds the {args.limit:.0f} deg per-call "
                "limit; make several smaller moves and watch the jaws"
            )

    if args.command == "autozero":
        if args.step <= 0 or args.settle <= 0 or args.travel <= 0:
            parser.error("--step, --settle and --travel must be positive")
        if not 0.0 < args.stall_ratio < 1.0:
            parser.error("--stall-ratio must be between 0 and 1")
        if args.stalls < 1:
            parser.error("--stalls must be at least 1")

    handlers = {"scan": scan, "read": read, "measure": measure,
                "jog": jog, "autozero": autozero, "verify": verify}
    try:
        return handlers[args.command](args)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
