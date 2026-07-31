#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

from ...constants import (
    DEFAULT_FIRST_ACTION_MAX_JUMP_DEG,
    DEFAULT_GRIPPER_MAX_ANGLE,
    DEFAULT_GRIPPER_MIN_ANGLE,
    DEFAULT_SPEED_MODE,
    DEFAULT_TCP_IP,
    DEFAULT_TCP_MAX_ATTEMPTS,
    DEFAULT_TCP_MAX_MESSAGE_BYTES,
    DEFAULT_TCP_PORT,
    DEFAULT_TCP_TIMEOUT_S,
    SPEED_MODES,
)


def _default_speed_config() -> dict[str, list[float]]:
    # 各速度模式下 6 个关节电机的最大速度，请不要随意更改。
    # 出处：恩培 fork EnpeiFollower.speed_config
    # （record = [90,40,100,100,100,10];inference = record × 0.7;teleop = [90,50,200,200,200,100]）。
    # "record_forpi" 是 fork 里漏登记的模式，先取与 record 相同，待厂商确认。
    return {
        "record": [90, 40, 100, 100, 100, 10],
        "inference": [63, 28, 70, 70, 70, 7],  # = record × 0.7
        "teleop": [90, 50, 200, 200, 200, 100],
        "record_forpi": [90, 40, 100, 100, 100, 10],  # 待厂商确认，暂与 record 相同
    }


def _default_filter_alphas() -> dict[str, float]:
    # 一阶低通滤波(EMA)系数，按速度模式分档。出处：恩培 fork EnpeiFollower.filter_config。
    # "record_forpi" 待厂商确认，暂与 record 相同。
    return {
        "record": 0.1,
        "inference": 0.1,
        "teleop": 0.1,
        "record_forpi": 0.1,  # 待厂商确认，暂与 record 相同
    }


def _default_joint_angle_range() -> dict[str, list[float]]:
    # 关节绝对量程（度），send_action 时做绝对钳制。
    # 出处：恩培 fork EnpeiLeader.SERVO_DEGREE_RANGE（主臂舵机量程，主从 1:1 映射，
    # 从臂实际量程待厂商确认）。夹爪量程单独由 gripper_min/max_angle 控制。
    return {
        "joint1": [0, 340],
        "joint2": [0, 180],
        "joint3": [0, 163],
        "joint4": [0, 335],
        "joint5": [0, 220],
        "joint6": [0, 335],
    }


@RobotConfig.register_subclass("episode1_follower")
@dataclass
class Episode1FollowerConfig(RobotConfig):
    # 上位机（Episode1 本体控制器）TCP 地址
    ip_address: str = DEFAULT_TCP_IP
    port: int = DEFAULT_TCP_PORT
    # TCP 收发超时（秒）、长度前缀上限（字节，防畸形包撑爆内存）、单命令最大尝试次数
    tcp_timeout: float = DEFAULT_TCP_TIMEOUT_S
    tcp_max_message_bytes: int = DEFAULT_TCP_MAX_MESSAGE_BYTES
    tcp_max_attempts: int = DEFAULT_TCP_MAX_ATTEMPTS

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # 单帧目标相对当前观测的最大增量（度）。官方 so_follower 默认 None（不钳制），
    # 这里默认 30° 开启相对增量钳制（防炸）；设为 None 可关闭。
    max_relative_target: float | None = 30.0

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # 速度模式：record(默认,最低速最稳) / inference / teleop / record_forpi。
    # draccus 自动暴露为 --robot.speed_mode=xxx，不用改官方脚本。
    speed_mode: str = DEFAULT_SPEED_MODE
    # 是否使用弧度（默认角度）。为 True 时关节角按弧度收发、夹爪按 [0,1] 收发。
    use_radian: bool = False
    # 各速度模式下的电机最大速度 / EMA 滤波系数（见上面 default_factory 注释）
    speed_config: dict[str, list[float]] = field(default_factory=_default_speed_config)
    filter_alphas: dict[str, float] = field(default_factory=_default_filter_alphas)
    # 夹爪 EMA 滤波系数。出处：恩培 fork filter_alpha_gripper = 0.9。
    filter_alpha_gripper: float = 0.9

    # 各关节减速比（观测角度→脉冲换算用）。出处：恩培 fork gear_ratios。
    gear_ratios: list[float] = field(default_factory=lambda: [25, 20, 25, 10, 4, 1])
    # 关节绝对量程（度），见上面 default_factory 注释
    joint_angle_range: dict[str, list[float]] = field(default_factory=_default_joint_angle_range)
    # 夹爪量程（度）。出处：恩培 fork EnpeiLeader 类常量 20/110（"根据自己夹爪实际量程设置"）。
    gripper_min_angle: float = DEFAULT_GRIPPER_MIN_ANGLE
    gripper_max_angle: float = DEFAULT_GRIPPER_MAX_ANGLE

    # 首帧防跳变阈值（度）：connect 后第一次 send_action，目标与 connect 时记录的
    # 从臂位姿差异超过此值则拒绝开始，提示把主臂摆到接近从臂的姿态。
    first_action_max_jump_deg: float = DEFAULT_FIRST_ACTION_MAX_JUMP_DEG

    # connect 时的归位动作（度）。出处：恩培 fork connect() 里写死的 fixed_degrees。
    # 第二段的第 5 关节 = 110-90 = 20（原代码 110-90 的现场计算）。
    move_to_startup_on_connect: bool = True
    startup_joint_positions: list[float] = field(
        default_factory=lambda: [180, 90, 83, 210, 110, 210, 90]
    )
    startup_joint_positions_2: list[float] = field(
        default_factory=lambda: [180, 90, 83, 210, 20, 210, 90]
    )
    startup_gripper_angle: int = 10

    # get_observation 读角度失败时的重试次数。出处：恩培 fork 写死 max_retries = 3。
    observation_max_retries: int = 3

    def __post_init__(self):
        super().__post_init__()
        if self.speed_mode not in SPEED_MODES:
            raise ValueError(
                f"非法 speed_mode: {self.speed_mode!r}。合法值: {list(SPEED_MODES)}"
                "（record_forpi 参数待厂商确认，暂与 record 相同）"
            )
        for table_name, table in (("speed_config", self.speed_config), ("filter_alphas", self.filter_alphas)):
            if self.speed_mode not in table:
                raise ValueError(
                    f"{table_name} 缺少 speed_mode {self.speed_mode!r} 的参数，现有键: {list(table)}"
                )
        if len(self.gear_ratios) != 6:
            raise ValueError(f"gear_ratios 必须有 6 个元素，当前: {self.gear_ratios}")
        if not self.gripper_min_angle < self.gripper_max_angle:
            raise ValueError(
                f"gripper_min_angle({self.gripper_min_angle}) 必须小于 gripper_max_angle({self.gripper_max_angle})"
            )
