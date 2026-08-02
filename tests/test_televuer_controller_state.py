import asyncio
import multiprocessing
from types import SimpleNamespace

from televuer.televuer import TeleVuer


def make_tvuer_without_server() -> TeleVuer:
    tvuer = TeleVuer.__new__(TeleVuer)
    ctx = multiprocessing.get_context("fork")
    tvuer.left_arm_pose_shared = ctx.Array("d", 16, lock=True)
    tvuer.right_arm_pose_shared = ctx.Array("d", 16, lock=True)
    tvuer.controller_event_time_ns_shared = ctx.Value("q", 0, lock=True)
    for prefix in ("left", "right"):
        setattr(tvuer, f"{prefix}_trigger_state_shared", ctx.Value("b", False, lock=True))
        setattr(tvuer, f"{prefix}_trigger_value_shared", ctx.Value("d", 0.0, lock=True))
        setattr(tvuer, f"{prefix}_squeeze_state_shared", ctx.Value("b", False, lock=True))
        setattr(tvuer, f"{prefix}_squeeze_value_shared", ctx.Value("d", 0.0, lock=True))
        setattr(tvuer, f"{prefix}_thumbstick_state_shared", ctx.Value("b", False, lock=True))
        setattr(tvuer, f"{prefix}_thumbstick_value_shared", ctx.Array("d", 2, lock=True))
        setattr(tvuer, f"{prefix}_aButton_shared", ctx.Value("b", False, lock=True))
        setattr(tvuer, f"{prefix}_bButton_shared", ctx.Value("b", False, lock=True))
    return tvuer


def controller_event(*, right_state: dict) -> SimpleNamespace:
    identity = [1.0, 0.0, 0.0, 0.0] * 4
    return SimpleNamespace(
        value={
            "left": identity,
            "right": identity,
            "leftState": {},
            "rightState": right_state,
        }
    )


def partial_controller_event(**value) -> SimpleNamespace:
    return SimpleNamespace(value=value)


def test_partial_trigger_event_does_not_release_held_right_grip() -> None:
    tvuer = make_tvuer_without_server()

    asyncio.run(
        tvuer.on_controller_move(
            controller_event(right_state={"squeeze": True, "squeezeValue": 1.0}), None
        )
    )
    asyncio.run(
        tvuer.on_controller_move(
            controller_event(right_state={"trigger": True, "triggerValue": 0.8}), None
        )
    )

    assert bool(tvuer.right_controller_squeeze_state) is True
    assert tvuer.right_controller_squeeze_value == 1.0
    assert tvuer.right_controller_trigger_value == 0.8


def test_right_only_pose_event_updates_tracking_without_left_controller() -> None:
    tvuer = make_tvuer_without_server()
    identity = [1.0, 0.0, 0.0, 0.0] * 4

    asyncio.run(
        tvuer.on_controller_move(
            partial_controller_event(
                right=identity,
                rightState={"squeeze": True, "squeezeValue": 0.75},
            ),
            None,
        )
    )

    assert tvuer.controller_event_time_ns_shared.value > 0
    assert list(tvuer.right_arm_pose_shared[:]) == identity
    assert tvuer.right_controller_squeeze_value == 0.75


def test_button_only_event_preserves_pose_freshness_timestamp() -> None:
    tvuer = make_tvuer_without_server()
    tvuer.controller_event_time_ns_shared.value = 123

    asyncio.run(
        tvuer.on_controller_move(
            partial_controller_event(rightState={"trigger": True, "triggerValue": 0.5}),
            None,
        )
    )

    assert tvuer.controller_event_time_ns_shared.value == 123
    assert tvuer.right_controller_trigger_value == 0.5


def test_invalid_left_pose_does_not_discard_valid_right_controller() -> None:
    tvuer = make_tvuer_without_server()
    identity = [1.0, 0.0, 0.0, 0.0] * 4

    asyncio.run(
        tvuer.on_controller_move(
            partial_controller_event(
                left=b"\x00",
                right=identity,
                leftState=None,
                rightState={"squeeze": True, "squeezeValue": 1.0},
            ),
            None,
        )
    )

    assert tvuer.controller_event_time_ns_shared.value > 0
    assert list(tvuer.right_arm_pose_shared[:]) == identity
    assert tvuer.right_controller_squeeze_value == 1.0
