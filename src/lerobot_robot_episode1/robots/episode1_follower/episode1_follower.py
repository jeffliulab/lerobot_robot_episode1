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

import logging
import time
from functools import cached_property
from typing import Any

import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_episode1_follower import Episode1FollowerConfig
from .episode_client import EpisodeClient, EpisodeClientError

logger = logging.getLogger(__name__)

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MOTOR_NAMES = JOINT_NAMES + ["gripper"]


class Episode1Follower(Robot):
    """
    Episode1 follower arm（恩培 Episode1 本体，经 TCP 上位机控制）
    """

    config_class = Episode1FollowerConfig
    name = "episode1_follower"

    def __init__(self, config: Episode1FollowerConfig):
        super().__init__(config)
        self.config = config
        self.controller: EpisodeClient | None = None
        self.episode1_is_connected = False

        self.cameras = make_cameras_from_configs(config.cameras)

        # 是否使用弧度（默认使用角度）
        self.use_radian = self.config.use_radian

        self.gear_ratios = self.config.gear_ratios

        # 当前观测到的关节角度对应的脉冲数
        self.current_joint_pulses = [0, 0, 0, 0, 0, 0]
        # 最近一次观测到的关节角度（度），相对增量钳制的基准
        self._last_obs_degrees: list[float] | None = None

        # 上次的观测到的夹爪角度
        self.last_observation_gripper = 0

        # 初始化一阶低通滤波器
        self.filtered_angles = [0.0] * 6
        # 是否是第一次接收角度
        self.is_first_action = [True] * 6
        # 滤波器参数，alpha值越小滤波效果越强（0-1之间）
        self.filter_alpha = self.config.filter_alphas[self.config.speed_mode]

        # 初始化一阶低通滤波器（夹爪）
        self.filtered_angles_gripper = 0.0
        # 是否是第一次接收角度
        self.is_first_action_gripper = True
        self.filter_alpha_gripper = self.config.filter_alpha_gripper

        # 初始化电机最大速度
        self.motors_max_speed = np.array(self.config.speed_config[self.config.speed_mode], dtype=float)

        # 首帧防跳变：connect 时记录的从臂位姿（度）与"是否还在等首帧"
        self._connect_pose_deg: list[float] | None = None
        self._awaiting_first_action = False

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in MOTOR_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.episode1_is_connected
            and self.controller is not None
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def _read_joint_angles_with_retry(self) -> list[float]:
        """读 6 个关节角度（度）；列表为 None 或任一元素为 None 时重试，
        超过 observation_max_retries 次仍失败则抛异常（绝不返回脏数据）。"""
        last_error: Exception | None = None
        for retry in range(self.config.observation_max_retries):
            try:
                obs_list = self.controller.get_motor_angles()
            except EpisodeClientError as e:
                last_error = e
                obs_list = None
            if obs_list is not None and all(v is not None for v in obs_list[:6]):
                return list(obs_list[:6])
            logger.info(f"读取电机角度失败，重试 {retry + 1}/{self.config.observation_max_retries}")
            time.sleep(0.01)  # 短暂延迟后重试
        raise EpisodeClientError(
            f"连续 {self.config.observation_max_retries} 次读取电机角度均失败，请检查上位机/CAN 总线连接"
        ) from last_error

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.episode1_is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # Initialize the controller
        try:
            self.controller = EpisodeClient(
                ip=self.config.ip_address,
                port=self.config.port,
                timeout=self.config.tcp_timeout,
                max_message_bytes=self.config.tcp_max_message_bytes,
                max_attempts=self.config.tcp_max_attempts,
            )
            logger.info("Connected to Episode1 controller")
            self.controller.sync_motor_angles()

            if self.config.move_to_startup_on_connect:
                # 移到默认位置
                t = self.controller.angle_mode(list(self.config.startup_joint_positions))
                if isinstance(t, (int, float)):
                    time.sleep(t)
                t = self.controller.angle_mode(list(self.config.startup_joint_positions_2))
                if isinstance(t, (int, float)):
                    time.sleep(t)
                # 夹爪运动到 startup_gripper_angle
                self.controller.servo_gripper(self.config.startup_gripper_angle)
                self.last_observation_gripper = self.config.startup_gripper_angle

            # 记录 connect 时的当前位姿，用于首帧防跳变与增量钳制基准
            self._connect_pose_deg = self._read_joint_angles_with_retry()
            self._last_obs_degrees = list(self._connect_pose_deg)
            self._update_joint_pulses(self._connect_pose_deg)
            self._awaiting_first_action = True

            self.episode1_is_connected = True
        except Exception:
            logger.exception("Failed to connect to Episode1 controller")
            if self.controller is not None:
                self.controller.close()
                self.controller = None
            raise

        for cam in self.cameras.values():
            cam.connect()

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        pass

    def _update_joint_pulses(self, joint_degrees: list[float]) -> None:
        """更新当前关节角度对应的脉冲数"""
        for i in range(6):
            self.current_joint_pulses[i] = int(3200 * self.gear_ratios[i] * joint_degrees[i] / 360)

    def get_observation(self) -> dict[str, Any]:
        if not self.episode1_is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()

        obs_list = self._read_joint_angles_with_retry()
        self._update_joint_pulses(obs_list)
        self._last_obs_degrees = list(obs_list)

        # 使用上次的夹爪角度（夹爪无法回读，用最后一次下发的值）
        obs_list.append(self.last_observation_gripper)

        # 如果使用弧度，则将角度转换为弧度
        if self.use_radian:
            obs_list = [angle * np.pi / 180.0 for angle in obs_list]

        obs_dict = {f"{motor}.pos": val for motor, val in zip(MOTOR_NAMES, obs_list, strict=True)}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def _check_first_action_jump(self, joint_goals_deg: dict[str, float]) -> None:
        """首帧防跳变：首帧目标与 connect 时记录的位姿差异超阈值则拒绝开始。"""
        if self._connect_pose_deg is None:
            return
        for i, motor in enumerate(JOINT_NAMES):
            goal = joint_goals_deg.get(motor)
            if goal is None:
                continue
            diff = abs(goal - self._connect_pose_deg[i])
            if diff > self.config.first_action_max_jump_deg:
                raise RuntimeError(
                    f"首帧动作与从臂当前位姿差异过大：{motor} 目标 {goal:.1f}°，"
                    f"当前 {self._connect_pose_deg[i]:.1f}°，差 {diff:.1f}° "
                    f"> 阈值 {self.config.first_action_max_jump_deg}°。"
                    "请把主臂摆到接近从臂的姿态再开始（防止从臂猛跳）。"
                )

    def _clamp_joints(self, joint_goals_deg: dict[str, float]) -> dict[str, float]:
        """双层钳制：相对增量钳制（官方 ensure_safe_goal_position）+ 绝对量程钳制。"""
        clamped = dict(joint_goals_deg)

        # 相对增量钳制：以最近一次观测为基准
        if self.config.max_relative_target is not None and self._last_obs_degrees is not None:
            goal_present = {
                f"{motor}.pos": (clamped[motor], self._last_obs_degrees[i])
                for i, motor in enumerate(JOINT_NAMES)
                if motor in clamped
            }
            if goal_present:
                safe = ensure_safe_goal_position(
                    goal_present, float(self.config.max_relative_target)
                )
                for i, motor in enumerate(JOINT_NAMES):
                    key = f"{motor}.pos"
                    if key in safe:
                        clamped[motor] = safe[key]

        # 绝对量程钳制：关节角度范围
        for motor, goal in clamped.items():
            lo, hi = self.config.joint_angle_range[motor]
            clamped[motor] = min(max(goal, lo), hi)
        return clamped

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            DeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.episode1_is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        action = dict(action)  # 不原地改调用方的字典
        goal_pos: dict[str, float] = {}

        # Extract motor positions from action dictionary
        joint_goals_deg: dict[str, float] = {}
        for i, motor in enumerate(JOINT_NAMES, 1):
            key = f"{motor}.pos"
            if key not in action:
                continue
            angle = action[key]

            # 如果使用弧度，先转换为角度再应用滤波
            # 这样可以保证滤波效果一致，因为弧度值比角度值小很多
            angle_in_degrees = angle * 180.0 / np.pi if self.use_radian else angle
            joint_goals_deg[motor] = angle_in_degrees

        # 首帧防跳变（只查第一帧，用未滤波的原始目标值）
        if self._awaiting_first_action:
            self._check_first_action_jump(joint_goals_deg)
            self._awaiting_first_action = False

        # 应用一阶低通滤波 (EMA)在角度值上
        for i, motor in enumerate(JOINT_NAMES, 1):
            if motor not in joint_goals_deg:
                continue
            if self.is_first_action[i - 1]:
                self.filtered_angles[i - 1] = joint_goals_deg[motor]
                self.is_first_action[i - 1] = False
            else:
                self.filtered_angles[i - 1] = (
                    self.filter_alpha * joint_goals_deg[motor]
                    + (1 - self.filter_alpha) * self.filtered_angles[i - 1]
                )

        filtered_deg = {
            motor: round(self.filtered_angles[i - 1], 2)
            for i, motor in enumerate(JOINT_NAMES, 1)
            if motor in joint_goals_deg
        }

        # 双层钳制（相对增量 + 绝对量程）
        clamped_deg = self._clamp_joints(filtered_deg)

        for i, motor in enumerate(JOINT_NAMES, 1):
            if motor not in clamped_deg:
                continue
            key = f"{motor}.pos"
            # 更新action中的值（保持与输入相同的单位，角度或弧度）
            action[key] = clamped_deg[motor] * np.pi / 180.0 if self.use_radian else clamped_deg[motor]
            # goal_pos 中始终使用角度值，用于发送给上位机
            goal_pos[str(i)] = clamped_deg[motor]

        # 夹爪单独控制
        if "gripper.pos" in action:
            angle = action["gripper.pos"]

            # 如果使用弧度，则将 [0, 1] 转回角度
            if self.use_radian:
                angle = angle * (self.config.gripper_max_angle - self.config.gripper_min_angle) + self.config.gripper_min_angle

            # 应用一阶低通滤波 (EMA)在角度值上
            if self.is_first_action_gripper:
                self.filtered_angles_gripper = angle
                self.is_first_action_gripper = False
            else:
                self.filtered_angles_gripper = (
                    self.filter_alpha_gripper * angle
                    + (1 - self.filter_alpha_gripper) * self.filtered_angles_gripper
                )

            # 绝对量程钳制：夹爪量程，取整数
            filtered_angle = int(
                min(
                    max(self.filtered_angles_gripper, self.config.gripper_min_angle),
                    self.config.gripper_max_angle,
                )
            )

            # 添加阈值，减少不必要的命令
            if abs(self.last_observation_gripper - filtered_angle) > 0:
                # 发送夹爪角度到夹爪舵机
                self.controller.servo_gripper(filtered_angle)
                # 需要保存一个变量给get_observation
                self.last_observation_gripper = filtered_angle

            # 更新action中的值为滤波后的值，但保持单位与输入相同
            if self.use_radian:
                # 转到 [0, 1]
                action["gripper.pos"] = (filtered_angle - self.config.gripper_min_angle) / (
                    self.config.gripper_max_angle - self.config.gripper_min_angle
                )
            else:
                action["gripper.pos"] = filtered_angle

        # 使用controller中的dynamic_move函数来处理关节运动
        if goal_pos:
            self.controller.dynamic_move(
                goal_pos=goal_pos,  # 目标角度
                current_joint_pulses=self.current_joint_pulses,  # 当前观测角度脉冲
                motors_max_speed=self.motors_max_speed.tolist(),  # 电机最大速度
            )

        return action

    def disconnect(self) -> None:
        """幂等断开：重复调用安全，内部异常只记日志不外抛（保证 Ctrl+C 路径总能走完）。"""
        if not self.episode1_is_connected and self.controller is None:
            return

        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception:
                logger.exception(f"断开相机时出错（已吞掉，不影响清理流程）")

        if self.controller is not None:
            # 因为dynamic_move是开环控制，所以在断开连接时，需要同步一次角度
            try:
                self.controller.sync_motor_angles()
            except Exception:
                logger.exception("disconnect 时 sync_motor_angles 失败（已吞掉）")
            try:
                self.controller.close()
            except Exception:
                logger.exception("disconnect 时关闭 TCP 连接失败（已吞掉）")
            self.controller = None

        self.episode1_is_connected = False
        logger.info(f"{self} disconnected.")
