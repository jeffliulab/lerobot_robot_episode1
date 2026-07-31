"""motors 层读写路径测试（fake scservo_sdk，无硬件）。"""

import numpy as np
import pytest

from conftest import FakeScservoModule
from lerobot_robot_episode1.motors.configs import FeetechMotorsBusConfig
from lerobot_robot_episode1.motors.feetech import FeetechMotorsBus
from lerobot_robot_episode1.motors.servo_controller import FeetechController


def _make_bus():
    config = FeetechMotorsBusConfig(
        port="/dev/fake",
        motors={f"motor{i}": (i, "sts3215") for i in range(1, 8)},
    )
    return FeetechMotorsBus(config)


def test_bus_read_write_path(fake_scservo):
    bus = _make_bus()
    bus.connect()
    assert bus.is_connected

    positions = bus.read("Present_Position")
    assert isinstance(positions, np.ndarray)
    assert positions.tolist() == [2048] * 7

    FakeScservoModule.POSITIONS[3] = 1000
    pos = bus.read("Present_Position", ["motor3"])
    assert pos.tolist() == [1000]

    bus.write("Goal_Position", 3000, ["motor1", "motor2"])
    assert FakeScservoModule.WRITES, "写路径没有产生任何 GroupSyncWrite"
    assert set(FakeScservoModule.WRITES[-1].keys()) == {1, 2}

    bus.disconnect()
    assert not bus.is_connected


def test_mock_mode_removed(fake_scservo):
    # fork 的 mock 分支 import 的是 lerobot 仓内部测试模块，已删除；这里明确报错
    config = FeetechMotorsBusConfig(port="/dev/fake", motors={"motor1": (1, "sts3215")}, mock=True)
    with pytest.raises(NotImplementedError, match="mock"):
        FeetechMotorsBus(config).connect()


def test_controller_read_positions(fake_scservo):
    controller = FeetechController(port="/dev/fake", motor_range=(1, 7))
    controller.connect()
    assert controller.is_connected

    positions = controller.read_positions()
    assert positions.tolist() == [2048] * 7

    # 单颗读取（带缓存）
    assert controller.read_position(1) == 2048
    # 超出范围的 ID 返回 None 而不是抛异常
    assert controller.read_position(8) is None

    batch = controller.batch_read_positions()
    assert batch == {i: 2048 for i in range(1, 8)}

    controller.set_positions([3000] * 7)
    assert FakeScservoModule.WRITES

    controller.disconnect()
    assert not controller.is_connected
