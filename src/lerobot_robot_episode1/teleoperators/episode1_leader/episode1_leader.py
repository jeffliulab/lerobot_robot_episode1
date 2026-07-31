#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified by Jeff (2026): ported from enpeizhao/lerobot_single_student — renamed enpei→episode1, adapted to lerobot 0.6.x plugin API

import glob
import json
import logging
import time
from importlib.resources import files
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ...motors.servo_controller import FeetechController
from .config_episode1_leader import Episode1LeaderConfig

logger = logging.getLogger(__name__)

MOTOR_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


def _reference_calibration_path() -> Path:
    """打包的参考零点 json（恩培那台臂实测值，只作初始参考）。"""
    return Path(str(files("lerobot_robot_episode1").joinpath("data/motor_calibration_reference.json")))


def load_reference_calibration() -> dict[int, int]:
    with open(_reference_calibration_path()) as f:
        return {int(k): v for k, v in json.load(f).items()}


class Episode1Leader(Teleoperator):
    """
    [Episode1 leader arm](https://enpeicv.com/) designed by EnpeiCV —— 7 舵机遥操作主臂。
    """

    config_class = Episode1LeaderConfig
    name = "episode1_leader"

    def __init__(self, config: Episode1LeaderConfig):
        super().__init__(config)
        self.config = config

        self.bus = FeetechController(
            port=self.config.port,
            motor_model=self.config.motor_model,
            motor_range=self.config.motor_id_range,
        )

        # 零点参考：优先用按 robot id 存放的校准文件（官方 calibration_dir 机制）；
        # 没有校准文件时退到打包的参考零点并 warning（那是恩培那台臂的实测值，不是你的臂）。
        if self.calibration:
            self.zero_references = {int(k): v for k, v in self.calibration.items()}
        else:
            self.zero_references = load_reference_calibration()
            logger.warning(
                f"未找到 id={self.id} 的校准文件，暂用包内参考零点（恩培演示臂实测值）。"
                "请运行 `episode1-set-middle` 或在 connect 时校准，为你的臂生成专属零点。"
            )

        # 是否使用弧度（默认角度）
        self.use_radian = self.config.use_radian

        # 初始化一阶低通滤波器
        self.filtered_angles = [0.0] * 7
        # 是否是第一次读取角度
        self.is_first_reading = [True] * 7
        # 滤波器参数，alpha值越小滤波效果越强（0-1之间）
        self.filter_alpha = self.config.filter_alphas[self.config.speed_mode]

        # 主臂夹爪 → 从臂夹爪的映射比例
        self._gripper_mapping_ratio = (
            (self.config.gripper_max_angle - self.config.gripper_min_angle)
            / self.config.leader_gripper_travel
        )

        # 防炸：读失败策略状态
        self._last_valid_action: dict[str, float] | None = None
        self._consecutive_read_failures = 0

    # ---- 校准（官方 calibration_dir + id 机制；格式是简单的 {motor_id: 零点脉冲值}）----

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f:
            self.calibration = {int(k): v for k, v in json.load(f).items()}

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        fpath.parent.mkdir(parents=True, exist_ok=True)
        with open(fpath, "w") as f:
            json.dump({str(k): v for k, v in self.calibration.items()}, f, indent=4)

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibration)

    def calibrate(self) -> None:
        """交互式校准（照官方 so_leader 的确认模式）。"""
        user_input = input(
            f"按回车使用包内参考零点并保存为 id={self.id} 的校准文件；"
            "输入 'c' 再回车，把当前臂姿态的原始读数采集为零点（请先把臂摆到参考姿态）: "
        )
        if user_input.strip().lower() == "c":
            raw = self.bus.batch_read_positions()
            self.calibration = {int(k): int(v) for k, v in raw.items()}
        else:
            self.calibration = load_reference_calibration()
        self.zero_references = dict(self.calibration)
        self._save_calibration()
        print(f"校准已保存到 {self.calibration_fpath}")

    # ---- 连接 ----

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def _check_port_exists(self) -> None:
        """防呆：串口不存在时列出当前可用串口再报错。"""
        if Path(self.config.port).exists():
            return
        available = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        raise DeviceNotConnectedError(
            f"串口 {self.config.port} 不存在。"
            f"当前可用串口: {available if available else '(无 /dev/ttyACM* 或 /dev/ttyUSB*，请检查 USB 线)'}"
        )

    def _scan_bus(self) -> None:
        """防呆：总线上必须恰好是 motor_id_range 里每颗 ID 各一颗，否则拒绝连接。"""
        start, end = self.config.motor_id_range
        bad_ids: list[int] = []
        for motor_id in range(start, end + 1):
            try:
                self.bus.motors_bus.read_with_motor_ids(
                    [self.config.motor_model], [motor_id], "ID", num_retry=2
                )
            except Exception:
                bad_ids.append(motor_id)
        if bad_ids:
            raise DeviceNotConnectedError(
                f"总线体检失败：ID {bad_ids} 无响应。"
                "可能原因：漏插该电机、串联线断开、或两颗电机 ID 重复（冲突同样表现为读取失败）。"
                "请检查连线与电机 ID 后重试。"
            )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self._check_port_exists()
        self.bus.connect()
        try:
            motors_bus = getattr(self.bus, "motors_bus", None)
            if motors_bus is not None and motors_bus.port_handler is not None:
                motors_bus.set_bus_baudrate(self.config.baudrate)
            self._scan_bus()
        except Exception:
            # 体检失败要释放串口，否则下次 connect 报 already connected
            try:
                self.bus.disconnect()
            except Exception:
                logger.exception("体检失败后释放串口时出错")
            raise

        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        pass

    # ---- 读动作 ----

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in MOTOR_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    def normalize_to_degrees(self, position, motor_id, zero_references):
        """
        Convert raw position value to degrees in -180 to +180 range
        Based on the apply_calibration function in feetech.py
        """
        if zero_references.get(motor_id) is not None:
            if position < 0:
                position = 0
            if position > 4095:
                position = 4095
            # Calculate relative position from zero reference
            relative_position = position - zero_references[motor_id]
            if self.config.rotation_direction[motor_id] == -1:
                relative_position = -relative_position
            degrees = round(
                relative_position * 360.0 / 4096 + self.config.reference_positions[motor_id], 2
            )

            if motor_id in self.config.degree_range:
                min_degree, max_degree = self.config.degree_range[motor_id]
                if degrees < min_degree:
                    degrees = min_degree
                elif degrees > max_degree:
                    degrees = max_degree

            return degrees

        # 如果没有零点参考，则返回 None
        return None

    def _map_gripper(self, angle_value: float) -> float:
        """把主臂夹爪角度映射到从臂夹爪量程 [gripper_min_angle, gripper_max_angle]。"""
        mapped = self.config.gripper_min_angle + self._gripper_mapping_ratio * angle_value
        # 如果超出范围，限制在 [gripper_min_angle, gripper_max_angle]
        mapped = min(max(mapped, self.config.gripper_min_angle), self.config.gripper_max_angle)
        if self.use_radian:
            # 映射到 [0, 1]
            return (mapped - self.config.gripper_min_angle) / (
                self.config.gripper_max_angle - self.config.gripper_min_angle
            )
        return int(mapped)

    def _read_action_once(self) -> dict[str, float] | None:
        """读一帧动作；任一舵机读不出则返回 None（由调用方决定降级策略）。"""
        action: dict[str, float] = {}
        for i, motor_name in enumerate(MOTOR_NAMES, 1):
            position = self.bus.read_position(i)
            angle = self.normalize_to_degrees(position, i, self.zero_references) if position is not None else None
            if angle is None:
                return None

            # 应用一阶低通滤波 (EMA)
            if self.is_first_reading[i - 1]:
                self.filtered_angles[i - 1] = angle
                self.is_first_reading[i - 1] = False
            else:
                self.filtered_angles[i - 1] = (
                    self.filter_alpha * angle + (1 - self.filter_alpha) * self.filtered_angles[i - 1]
                )

            # 使用滤波后的角度，保留2位小数角度
            angle_value = round(self.filtered_angles[i - 1], 2)

            if i == 7:  # 夹爪单独映射
                action[f"{motor_name}.pos"] = self._map_gripper(angle_value)
            else:
                # 如果使用弧度，则将角度转换为弧度
                if self.use_radian:
                    angle_value = angle_value * np.pi / 180.0
                action[f"{motor_name}.pos"] = angle_value
        return action

    def get_action(self) -> dict[str, float]:
        """防炸：任一舵机读不出 → 返回上一帧有效值 + warning；
        连续失败超过 max_consecutive_read_failures → 抛异常走安全断开。
        绝不返回 0/None 让从臂收到垃圾动作。"""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        try:
            action = self._read_action_once()
        except Exception as e:
            logger.warning(f"读取主臂动作异常: {e}")
            action = None

        if action is None:
            self._consecutive_read_failures += 1
            if self._consecutive_read_failures >= self.config.max_consecutive_read_failures:
                raise ConnectionError(
                    f"主臂连续 {self._consecutive_read_failures} 次读舵机失败，拒绝继续输出动作。"
                    "请检查主臂串联线/供电后重新连接。"
                )
            if self._last_valid_action is None:
                raise ConnectionError(
                    "主臂连接后从未读到有效帧，无法输出动作。请检查主臂串联线/供电。"
                )
            logger.warning(
                f"本帧读舵机失败（连续第 {self._consecutive_read_failures} 次），返回上一帧有效值。"
            )
            return dict(self._last_valid_action)

        self._consecutive_read_failures = 0
        self._last_valid_action = dict(action)

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        """幂等断开：重复调用安全，内部异常只记日志不外抛（保证 Ctrl+C 路径总能走完）。"""
        if not self.is_connected:
            return
        try:
            self.bus.disconnect()
        except Exception:
            logger.exception(f"{self} disconnect 时出错（已吞掉，不影响清理流程）")
        logger.info(f"{self} disconnected.")
