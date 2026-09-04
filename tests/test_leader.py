"""Episode1Leader 测试：connect 体检、读失败策略、校准存取。全部无硬件（fake bus）。"""

import json

import pytest
from lerobot.utils.errors import DeviceNotConnectedError

from conftest import FakeLeaderBus
from lerobot_robot_episode1.teleoperators.episode1_leader import (
    Episode1Leader,
    Episode1LeaderConfig,
)
from lerobot_robot_episode1.teleoperators.episode1_leader.episode1_leader import (
    load_reference_calibration,
)


def _make_leader(tmp_path, fake_bus=None, **cfg_kwargs):
    cfg_kwargs.setdefault("port", "/dev/null")  # 存在但不串口，connect 只查存在性
    cfg_kwargs.setdefault("calibration_dir", tmp_path)
    leader = Episode1Leader(Episode1LeaderConfig(**cfg_kwargs))
    if fake_bus is not None:
        leader.bus = fake_bus
    return leader


# ------------------------------------------------------------ connect 体检（防呆）


def test_connect_refuses_missing_port(tmp_path):
    leader = _make_leader(tmp_path, port="/dev/episode1-nonexistent")
    with pytest.raises(DeviceNotConnectedError, match="不存在"):
        leader.connect(calibrate=False)


def test_connect_bus_scan_ok(tmp_path):
    leader = _make_leader(tmp_path, fake_bus=FakeLeaderBus())
    leader.connect(calibrate=False)
    assert leader.is_connected
    leader.disconnect()
    assert not leader.is_connected


def test_connect_bus_scan_reports_bad_ids(tmp_path):
    # ID 3 无响应 → 拒绝连接并报缺谁
    leader = _make_leader(tmp_path, fake_bus=FakeLeaderBus(fail_ids={3}))
    with pytest.raises(DeviceNotConnectedError, match=r"\[3\]"):
        leader.connect(calibrate=False)
    # 体检失败后串口要释放，允许重试
    assert not leader.is_connected


# ------------------------------------------------------------ get_action 读失败策略（防炸）


def test_get_action_success(tmp_path):
    leader = _make_leader(tmp_path, fake_bus=FakeLeaderBus())
    leader.connect(calibrate=False)
    action = leader.get_action()
    assert set(action) == {f"{m}.pos" for m in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]}
    assert all(isinstance(v, (int, float)) for v in action.values())


def test_get_action_never_returns_none(tmp_path):
    bus = FakeLeaderBus()
    leader = _make_leader(tmp_path, fake_bus=bus, max_consecutive_read_failures=2)
    leader.connect(calibrate=False)

    good = leader.get_action()
    # 一颗舵机读不出 → 返回上一帧有效值，绝不返回 None/0
    bus.positions[2] = None
    fallback = leader.get_action()
    assert fallback == good
    # 连续失败超阈值 → 抛异常走安全断开
    with pytest.raises(ConnectionError, match="连续 2"):
        leader.get_action()


def test_get_action_first_frame_failure_raises(tmp_path):
    bus = FakeLeaderBus()
    bus.positions[5] = None  # 从未读到过有效帧
    leader = _make_leader(tmp_path, fake_bus=bus)
    leader.connect(calibrate=False)
    with pytest.raises(ConnectionError, match="从未读到有效帧"):
        leader.get_action()


def test_get_action_gripper_mapping(tmp_path):
    # 主臂夹爪满行程 90° 应映射到从臂夹爪上限 100°（默认量程 [20, 100]；上限按本机实测红线 ≤100，2026-09-04 从 110 改）
    bus = FakeLeaderBus()
    leader = _make_leader(tmp_path, fake_bus=bus)
    leader.connect(calibrate=False)
    # 直接构造夹爪读数：zero_ref + 90°×4096/360，方向为 1；filter_alpha=1 关掉 EMA 便于断言
    leader.is_first_reading = [False] * 7
    leader.filtered_angles = [0.0] * 7
    leader.filter_alpha = 1.0
    zero = leader.zero_references[7]
    bus.positions[7] = zero + int(90 * 4096 / 360)
    action = leader.get_action()
    assert action["gripper.pos"] == 100


def test_get_action_radian_mode(tmp_path):
    leader = _make_leader(tmp_path, fake_bus=FakeLeaderBus(), use_radian=True)
    leader.connect(calibrate=False)
    action = leader.get_action()
    # 弧度模式：夹爪映射到 [0, 1]
    assert 0.0 <= action["gripper.pos"] <= 1.0


# ------------------------------------------------------------ 校准（官方 calibration_dir + id）


def test_reference_calibration_loads():
    ref = load_reference_calibration()
    assert set(ref) == set(range(1, 8))
    assert all(isinstance(v, int) for v in ref.values())


def test_calibration_file_roundtrip(tmp_path):
    leader = _make_leader(tmp_path, fake_bus=FakeLeaderBus(), id="arm_a")
    # 无校准文件 → is_calibrated False，zero_references 退到参考零点
    assert not leader.is_calibrated
    assert leader.zero_references == load_reference_calibration()

    leader.calibration = {i: 2000 + i for i in range(1, 8)}
    leader._save_calibration()
    assert leader.calibration_fpath.name == "arm_a.json"

    # 重建一个同 id 的 leader，应加载刚保存的校准
    leader2 = _make_leader(tmp_path, fake_bus=FakeLeaderBus(), id="arm_a")
    assert leader2.is_calibrated
    assert leader2.zero_references == {i: 2000 + i for i in range(1, 8)}


def test_calibration_bound_to_id(tmp_path):
    # id 对不上就不会默默复用别的臂的校准
    leader = _make_leader(tmp_path, fake_bus=FakeLeaderBus(), id="arm_a")
    leader.calibration = {i: 2000 for i in range(1, 8)}
    leader._save_calibration()
    other = _make_leader(tmp_path, fake_bus=FakeLeaderBus(), id="arm_b")
    assert not other.is_calibrated


# ------------------------------------------------------------ disconnect 幂等


def test_disconnect_idempotent_and_swallows_errors(tmp_path):
    class ExplodingBus(FakeLeaderBus):
        def disconnect(self):
            raise RuntimeError("boom")

    leader = _make_leader(tmp_path, fake_bus=ExplodingBus())
    leader.connect(calibrate=False)
    leader.disconnect()  # 内部异常不外抛
    leader.disconnect()  # 重复调用安全
