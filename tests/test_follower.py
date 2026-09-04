"""Episode1Follower 测试：双层钳制、首帧防跳变、弧度开关、disconnect 幂等。fake 上位机，无真机。"""

import numpy as np
import pytest
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

import lerobot_robot_episode1.robots.episode1_follower.episode1_follower as follower_mod
from conftest import FakeEpisodeController
from lerobot_robot_episode1.robots.episode1_follower import (
    Episode1Follower,
    Episode1FollowerConfig,
)
from lerobot_robot_episode1.robots.episode1_follower.episode_client import EpisodeClientError

POSE = [180, 90, 83, 210, 110, 210]  # fake 上位机回报的从臂位姿（度）


def _make_follower(tmp_path, monkeypatch, fake=None, **cfg_kwargs):
    cfg_kwargs.setdefault("calibration_dir", tmp_path)
    follower = Episode1Follower(Episode1FollowerConfig(**cfg_kwargs))
    fake = fake or FakeEpisodeController(angles=POSE)
    monkeypatch.setattr(follower_mod, "EpisodeClient", lambda **kwargs: fake)
    return follower, fake


def _connected_follower(tmp_path, monkeypatch, **cfg_kwargs):
    follower, fake = _make_follower(tmp_path, monkeypatch, **cfg_kwargs)
    follower.connect()
    return follower, fake


def _neutral_action():
    return {f"joint{i}.pos": float(POSE[i - 1]) for i in range(1, 7)}


# ------------------------------------------------------------ connect


def test_connect_records_pose_and_moves_to_startup(tmp_path, monkeypatch):
    follower, fake = _connected_follower(tmp_path, monkeypatch)
    assert follower.is_connected
    assert follower._connect_pose_deg == POSE
    assert fake.synced == 1
    # 归位两段 + 夹爪
    assert fake.angle_mode_calls == [
        [180, 90, 83, 210, 110, 210],
        [180, 90, 83, 210, 20, 210],
    ]  # 厂商 API 只收 6 个角，第 7 个不再发
    assert fake.sent_gripper == [10]


def test_double_connect_rejected(tmp_path, monkeypatch):
    follower, _ = _connected_follower(tmp_path, monkeypatch)
    with pytest.raises(DeviceAlreadyConnectedError):
        follower.connect()


# ------------------------------------------------------------ 首帧防跳变（防炸）


def test_first_action_jump_rejected(tmp_path, monkeypatch):
    follower, fake = _connected_follower(tmp_path, monkeypatch)
    action = _neutral_action()
    action["joint1.pos"] = POSE[0] + 40.0  # 差 40° > 默认阈值 30°
    with pytest.raises(RuntimeError, match="把主臂摆到接近从臂的姿态"):
        follower.send_action(action)
    assert not fake.dynamic_moves, "跳变帧绝不下发"


def test_first_action_within_threshold_passes(tmp_path, monkeypatch):
    follower, fake = _connected_follower(tmp_path, monkeypatch)
    action = _neutral_action()
    action["joint1.pos"] = POSE[0] + 20.0
    follower.send_action(action)
    assert fake.dynamic_moves, "阈值内的首帧应正常下发"
    # 检查只做一次：之后不再受限
    follower.send_action(_neutral_action())
    assert len(fake.dynamic_moves) == 2


# ------------------------------------------------------------ 双层钳制（防炸）


def test_relative_increment_clamped(tmp_path, monkeypatch):
    follower, fake = _connected_follower(
        tmp_path, monkeypatch, filter_alphas={m: 1.0 for m in ["record", "inference", "teleop", "record_forpi"]}
    )
    follower.send_action(_neutral_action())  # 过首帧检查
    action = _neutral_action()
    action["joint1.pos"] = POSE[0] + 100.0
    sent = follower.send_action(action)
    # max_relative_target 默认 30°：目标被钳到当前观测 +30
    assert sent["joint1.pos"] == pytest.approx(POSE[0] + 30.0)
    assert fake.dynamic_moves[-1]["1"] == pytest.approx(POSE[0] + 30.0)


def test_absolute_range_clamped(tmp_path, monkeypatch):
    follower, fake = _connected_follower(
        tmp_path,
        monkeypatch,
        max_relative_target=None,  # 关掉增量钳制，单独验证绝对量程
        filter_alphas={m: 1.0 for m in ["record", "inference", "teleop", "record_forpi"]},
    )
    follower.send_action(_neutral_action())  # 过首帧检查
    action = _neutral_action()
    action["joint2.pos"] = 999.0  # joint2 量程 [0, 180]
    sent = follower.send_action(action)
    assert sent["joint2.pos"] == pytest.approx(180.0)


def test_gripper_range_clamped(tmp_path, monkeypatch):
    follower, fake = _connected_follower(
        tmp_path, monkeypatch, filter_alpha_gripper=1.0
    )
    follower.send_action(_neutral_action())  # 过首帧检查
    sent = follower.send_action({"gripper.pos": 999.0})
    assert fake.sent_gripper[-1] == 100  # 夹爪量程上限（本机实测红线 ≤100）
    assert sent["gripper.pos"] == 100


# ------------------------------------------------------------ 弧度开关


def test_radian_mode(tmp_path, monkeypatch):
    follower, fake = _connected_follower(
        tmp_path,
        monkeypatch,
        use_radian=True,
        filter_alphas={m: 1.0 for m in ["record", "inference", "teleop", "record_forpi"]},
        filter_alpha_gripper=1.0,
    )
    action = {f"joint{i}.pos": float(np.deg2rad(POSE[i - 1])) for i in range(1, 7)}
    action["gripper.pos"] = 1.5  # [0,1] 归一化，超 1 → 钳到上限 100°
    sent = follower.send_action(action)
    assert fake.dynamic_moves[-1]["1"] == pytest.approx(POSE[0], abs=0.01)
    assert fake.sent_gripper[-1] == 100
    # 返回值保持弧度/[0,1] 单位
    assert sent["joint1.pos"] == pytest.approx(np.deg2rad(POSE[0]), abs=1e-3)
    assert sent["gripper.pos"] == pytest.approx(1.0)


def test_ema_filter_smooths(tmp_path, monkeypatch):
    # record 模式 alpha=0.1：第二帧只移动 10% 的偏差
    follower, fake = _connected_follower(tmp_path, monkeypatch)
    follower.send_action(_neutral_action())  # 首帧：filtered = 当前位姿
    action = _neutral_action()
    action["joint1.pos"] = POSE[0] + 10.0
    follower.send_action(action)
    assert fake.dynamic_moves[-1]["1"] == pytest.approx(POSE[0] + 1.0, abs=0.01)


# ------------------------------------------------------------ 观测与读失败


def test_get_observation(tmp_path, monkeypatch):
    follower, _ = _connected_follower(tmp_path, monkeypatch)
    obs = follower.get_observation()
    assert set(obs) == {f"{m}.pos" for m in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]}
    assert obs["joint1.pos"] == pytest.approx(POSE[0])
    assert obs["gripper.pos"] == 10  # 夹爪无法回读，用最后一次下发的值


def test_observation_read_failure_raises(tmp_path, monkeypatch):
    follower, fake = _connected_follower(tmp_path, monkeypatch, observation_max_retries=2)
    fake.get_motor_angles = lambda: None  # 上位机/CAN 故障
    with pytest.raises(EpisodeClientError, match="读取电机角度均失败"):
        follower.get_observation()


def test_not_connected_guards(tmp_path, monkeypatch):
    follower, _ = _make_follower(tmp_path, monkeypatch)
    with pytest.raises(DeviceNotConnectedError):
        follower.get_observation()
    with pytest.raises(DeviceNotConnectedError):
        follower.send_action(_neutral_action())


# ------------------------------------------------------------ disconnect（防炸：幂等 + 不外抛）


def test_disconnect_idempotent_and_swallows_errors(tmp_path, monkeypatch):
    follower, fake = _connected_follower(tmp_path, monkeypatch)

    def boom():
        raise RuntimeError("boom")

    fake.sync_motor_angles = boom
    fake.close = boom
    follower.disconnect()  # 内部异常只记日志
    assert not follower.is_connected
    follower.disconnect()  # 重复调用安全


def test_disconnect_syncs_angles(tmp_path, monkeypatch):
    follower, fake = _connected_follower(tmp_path, monkeypatch)
    follower.disconnect()
    # connect 1 次 + disconnect 收尾 1 次（dynamic_move 是开环，断开时必须同步一次）
    assert fake.synced == 2
    assert fake.closed
