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

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

from ...constants import (
    DEFAULT_BAUDRATE,
    DEFAULT_GRIPPER_MAX_ANGLE,
    DEFAULT_GRIPPER_MIN_ANGLE,
    DEFAULT_LEADER_GRIPPER_TRAVEL,
    DEFAULT_MAX_CONSECUTIVE_READ_FAILURES,
    DEFAULT_MOTOR_ID_RANGE,
    DEFAULT_SPEED_MODE,
    SPEED_MODES,
)


def _default_filter_alphas() -> dict[str, float]:
    # 一阶低通滤波(EMA)系数，按速度模式分档。出处：恩培 fork EnpeiLeader.filter_config
    # （三个模式都是 0.99；alpha 越小滤波越强）。
    # "record_forpi" 是 fork 里漏登记的模式，先取与 record 相同，待厂商确认。
    return {
        "record": 0.99,
        "inference": 0.99,
        "teleop": 0.99,
        "record_forpi": 0.99,  # 待厂商确认，暂与 record 相同
    }


def _default_rotation_direction() -> dict[int, int]:
    # 电机旋转方向 (1=顺时针增大角度, -1=顺时针减小角度)。
    # 出处：恩培 fork EnpeiLeader.SERVO_ROTATION_DIRECTION。
    return {1: -1, 2: 1, 3: 1, 4: -1, 5: 1, 6: -1, 7: 1}


def _default_degree_range() -> dict[int, list[float]]:
    # 各关节角度量程（度），用于读数钳制。出处：恩培 fork EnpeiLeader.SERVO_DEGREE_RANGE。
    return {
        1: [0, 340],
        2: [0, 180],
        3: [0, 163],
        4: [0, 335],
        5: [0, 220],
        6: [0, 335],
        7: [0, 110],  # 夹爪的角度范围
    }


def _default_reference_positions() -> dict[int, float]:
    # 电机零点参考位姿（度）：校准零点对应的"标准姿态"角度。
    # 出处：恩培 fork EnpeiLeader.SERVO_REFERENCE_POSITIONS。
    return {1: 180, 2: 90, 3: 83, 4: 210, 5: 110, 6: 210, 7: 90}


@TeleoperatorConfig.register_subclass("episode1_leader")
@dataclass
class Episode1LeaderConfig(TeleoperatorConfig):
    # 串口设备路径，如 /dev/ttyACM0
    port: str

    # 电机 ID 范围（闭区间），默认 (1, 7) 即 7 颗舵机
    motor_id_range: tuple[int, int] = DEFAULT_MOTOR_ID_RANGE
    # 舵机型号，恩培主臂用的是飞特 STS3215
    motor_model: str = "sts3215"
    # 总线波特率，飞特舵机默认 1Mbps
    baudrate: int = DEFAULT_BAUDRATE

    # 速度模式：record(默认,最稳) / inference / teleop / record_forpi。
    # draccus 自动暴露为 --teleop.speed_mode=xxx，不用改官方脚本。
    speed_mode: str = DEFAULT_SPEED_MODE
    # 是否输出弧度（默认角度）。为 True 时关节角转弧度、夹爪映射到 [0,1]。
    use_radian: bool = False
    # 各速度模式下的 EMA 滤波系数
    filter_alphas: dict[str, float] = field(default_factory=_default_filter_alphas)

    # 从臂夹爪量程（度），主臂夹爪角度线性映射到该区间后输出。
    # 根据自己夹爪的实际角度范围设置（恩培 fork 原注释）。
    gripper_min_angle: float = DEFAULT_GRIPPER_MIN_ANGLE
    gripper_max_angle: float = DEFAULT_GRIPPER_MAX_ANGLE
    # 主臂夹爪舵机的全行程角度（度），映射分母
    leader_gripper_travel: float = DEFAULT_LEADER_GRIPPER_TRAVEL

    # 连续读舵机失败上限：单次失败返回上一帧有效值，连续失败超过此值抛异常。
    max_consecutive_read_failures: int = DEFAULT_MAX_CONSECUTIVE_READ_FAILURES

    # 每颗电机的旋转方向 / 角度量程 / 零点参考位姿（见上面的 default_factory 注释）
    rotation_direction: dict[int, int] = field(default_factory=_default_rotation_direction)
    degree_range: dict[int, list[float]] = field(default_factory=_default_degree_range)
    reference_positions: dict[int, float] = field(default_factory=_default_reference_positions)

    def __post_init__(self):
        # 0.6.x 的 TeleoperatorConfig 没有 __post_init__（RobotConfig 才有），防御性调用
        parent_post_init = getattr(super(), "__post_init__", None)
        if parent_post_init is not None:
            parent_post_init()
        if self.speed_mode not in SPEED_MODES:
            raise ValueError(
                f"非法 speed_mode: {self.speed_mode!r}。合法值: {list(SPEED_MODES)}"
                "（record_forpi 参数待厂商确认，暂与 record 相同）"
            )
        if self.speed_mode not in self.filter_alphas:
            raise ValueError(
                f"filter_alphas 缺少 speed_mode {self.speed_mode!r} 的滤波系数，"
                f"现有键: {list(self.filter_alphas)}"
            )
        if not self.gripper_min_angle < self.gripper_max_angle:
            raise ValueError(
                f"gripper_min_angle({self.gripper_min_angle}) 必须小于 gripper_max_angle({self.gripper_max_angle})"
            )
        if self.motor_id_range[0] > self.motor_id_range[1]:
            raise ValueError(f"motor_id_range 非法: {self.motor_id_range}")
