"""episode1-set-middle --now：不进交互循环的中位校准（fake scservo_sdk，无硬件）。"""
import json

from conftest import FakeGroupSyncWrite, FakeScservoModule
from lerobot_robot_episode1.cli import set_middle
from lerobot_robot_episode1.motors.servo_controller import FeetechController

TORQUE_ENABLE_ADDR = 40   # sts3215 控制表：Torque_Enable=(40,1)；写 128 = 飞特「当前位置定义为 2048」


def _controller():
    c = FeetechController(port="/dev/fake", motor_range=(1, 7))
    c.connect()
    return c


def test_set_middle_now_writes_128_and_saves(fake_scservo, tmp_path, monkeypatch):
    monkeypatch.setattr(set_middle, "HF_LEROBOT_CALIBRATION", tmp_path)
    # 写入前是任意读数（含多圈/符号位那种超范围值），写入 128 后 fake 舵机应答 2048
    FakeScservoModule.POSITIONS = {1: 4320, 2: 4234, 3: 32943, 4: 220, 5: 1357, 6: 1903, 7: 2062}
    real_tx = FakeGroupSyncWrite.txPacket

    def tx_and_recenter(self):
        r = real_tx(self)
        if self.addr == TORQUE_ENABLE_ADDR and all(v == [128] for v in self.params.values()):
            for i in self.params: FakeScservoModule.POSITIONS[i] = 2048
        return r
    monkeypatch.setattr(FakeGroupSyncWrite, "txPacket", tx_and_recenter)

    c = _controller()
    assert set_middle.set_middle_now(c, (1, 7), "episode1_leader") is True

    torque_writes = [w for w in FakeScservoModule.WRITES if set(w) == set(range(1, 8)) and all(v == [128] for v in w.values())]
    assert torque_writes, "没有对 7 颗舵机写 Torque_Enable=128"
    f = tmp_path / set_middle.TELEOPERATORS / "episode1_leader" / "episode1_leader.json"
    assert f.exists(), "零点参考文件没写"
    assert json.loads(f.read_text()) == {str(i): 2048 for i in range(1, 8)}


def test_set_middle_now_refuses_when_not_centered(fake_scservo, tmp_path, monkeypatch):
    monkeypatch.setattr(set_middle, "HF_LEROBOT_CALIBRATION", tmp_path)
    FakeScservoModule.POSITIONS = {i: 4320 for i in range(1, 8)}   # 写了 128 读数也不变 → 视为失败
    c = _controller()
    assert set_middle.set_middle_now(c, (1, 7), "episode1_leader") is False
    assert not (tmp_path / set_middle.TELEOPERATORS / "episode1_leader" / "episode1_leader.json").exists()
