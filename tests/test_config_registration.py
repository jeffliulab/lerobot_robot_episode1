"""config 注册与校验测试：官方发现/工厂机制能认出 episode1_follower / episode1_leader。"""

import pytest
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config

import lerobot_robot_episode1  # noqa: F401  # import 即触发注册（官方发现机制也是这么加载的）
from lerobot_robot_episode1 import (
    Episode1Follower,
    Episode1FollowerConfig,
    Episode1Leader,
    Episode1LeaderConfig,
)


def test_choice_names_registered():
    assert RobotConfig.get_choice_name(Episode1FollowerConfig) == "episode1_follower"
    assert TeleoperatorConfig.get_choice_name(Episode1LeaderConfig) == "episode1_leader"


def test_make_robot_from_config(tmp_path):
    # 走官方工厂：未知名称 fallback 到按命名约定动态加载
    robot = make_robot_from_config(Episode1FollowerConfig(calibration_dir=tmp_path))
    assert isinstance(robot, Episode1Follower)
    assert not robot.is_connected


def test_make_teleoperator_from_config(tmp_path):
    teleop = make_teleoperator_from_config(Episode1LeaderConfig(port="/dev/null", calibration_dir=tmp_path))
    assert isinstance(teleop, Episode1Leader)
    assert not teleop.is_connected


def test_invalid_speed_mode_rejected(tmp_path):
    with pytest.raises(ValueError, match="非法 speed_mode"):
        Episode1FollowerConfig(speed_mode="turbo", calibration_dir=tmp_path)
    with pytest.raises(ValueError, match="非法 speed_mode"):
        Episode1LeaderConfig(port="/dev/null", speed_mode="turbo", calibration_dir=tmp_path)


def test_record_forpi_registered_with_record_values():
    # fork 的 bug 修复：record_forpi 补进字典，值暂与 record 相同（待厂商确认）
    cfg = Episode1FollowerConfig()
    assert "record_forpi" in cfg.speed_config
    assert cfg.speed_config["record_forpi"] == cfg.speed_config["record"]
    assert "record_forpi" in cfg.filter_alphas
    # record_forpi 是合法模式
    cfg2 = Episode1FollowerConfig(speed_mode="record_forpi")
    assert cfg2.speed_mode == "record_forpi"


def test_default_speed_mode_is_record():
    # 默认即最稳：最低速模式
    assert Episode1FollowerConfig().speed_mode == "record"
    assert Episode1LeaderConfig(port="/dev/null").speed_mode == "record"


def test_gripper_range_validation(tmp_path):
    with pytest.raises(ValueError, match="gripper_min_angle"):
        Episode1FollowerConfig(gripper_min_angle=200, gripper_max_angle=100, calibration_dir=tmp_path)
    with pytest.raises(ValueError, match="gripper_min_angle"):
        Episode1LeaderConfig(
            port="/dev/null", gripper_min_angle=200, gripper_max_angle=100, calibration_dir=tmp_path
        )
