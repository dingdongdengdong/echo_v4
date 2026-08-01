import logging
from dataclasses import asdict
from pprint import pformat

from lerobot.configs import parser
from lerobot.processor.factory import make_default_processors
from lerobot.robots.utils import make_robot_from_config
from lerobot.scripts.lerobot_record import RecordConfig, record
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleop_loop
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization

from .config import Quest2VuerConfig, RobopartyRightArmConfig
from .processor import make_quest_processor


def _validate_configs(robot_config, teleop_config) -> None:
    if not isinstance(robot_config, RobopartyRightArmConfig):
        raise TypeError("--robot.type must be roboparty_right_arm")
    if not isinstance(teleop_config, Quest2VuerConfig):
        raise TypeError("--teleop.type must be quest2_vuer")


@parser.wrap()
def _teleoperate(cfg: TeleoperateConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    _validate_configs(cfg.robot, cfg.teleop)
    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_processor = make_quest_processor(cfg.robot, cfg.teleop)
    _, robot_action_processor, observation_processor = make_default_processors()
    display_started = False
    try:
        if cfg.display_data:
            init_visualization(
                cfg.display_mode,
                session_name="roboparty-teleoperation",
                ip=cfg.display_ip,
                port=cfg.display_port,
            )
            display_started = True
        teleop.connect()
        robot.connect()
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            duration=cfg.teleop_time_s,
            display_data=cfg.display_data,
            display_mode=cfg.display_mode,
            display_compressed_images=cfg.display_compressed_images,
            teleop_action_processor=teleop_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=observation_processor,
        )
    except KeyboardInterrupt:
        pass
    finally:
        robot.disconnect()
        teleop.disconnect()
        if display_started:
            shutdown_visualization(cfg.display_mode)


@parser.wrap()
def _record(cfg: RecordConfig):
    if cfg.teleop is None:
        raise ValueError("recording requires --teleop.type=quest2_vuer")
    _validate_configs(cfg.robot, cfg.teleop)
    return record(cfg, teleop_action_processor=make_quest_processor(cfg.robot, cfg.teleop))


def teleoperate_main() -> None:
    _teleoperate()


def record_main() -> None:
    _record()
