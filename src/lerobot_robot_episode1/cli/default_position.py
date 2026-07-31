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
# (fork 的 src/lerobot/episode_default_position.py 改造而来:归位角度改从 config 默认值取,单一出处)

"""
让 Episode1 运动到默认位置，方便安装相机、夹爪。

使用方法：
episode1-default-position [--ip=IP地址] [--port=端口号]
"""

import argparse
import logging
import time

from ..constants import DEFAULT_TCP_IP, DEFAULT_TCP_PORT, DEFAULT_TCP_TIMEOUT_S
from ..robots.episode1_follower.config_episode1_follower import Episode1FollowerConfig
from ..robots.episode1_follower.episode_client import EpisodeClient

logger = logging.getLogger(__name__)

# 归位后夹爪角度。出处：恩培 fork episode_default_position.py 写死的 servo_gripper(40)
# （与 follower connect 里的 10 不同——这里是方便装夹爪的维护位姿）。
DEFAULT_MAINTENANCE_GRIPPER_ANGLE = 40


def move_to_default_position(ip: str, port: int, timeout: float, gripper_angle: int) -> None:
    """
    移动机械臂到默认位置

    参数:
        ip: 机器人控制器的IP地址
        port: 机器人控制器的端口号
        timeout: TCP 收发超时（秒）
        gripper_angle: 归位后的夹爪角度
    """
    # 归位角度与 follower config 默认值保持单一出处
    defaults = Episode1FollowerConfig()

    controller = EpisodeClient(ip=ip, port=port, timeout=timeout)
    logger.info(f"Connected to Episode1 controller at {ip}:{port}")
    try:
        # 移动到默认位置（两段，与 follower connect 的归位动作一致）
        t = controller.angle_mode(list(defaults.startup_joint_positions))
        if isinstance(t, (int, float)):
            time.sleep(t)
        t = controller.angle_mode(list(defaults.startup_joint_positions_2))
        if isinstance(t, (int, float)):
            time.sleep(t)

        # 夹爪运动到维护角度
        controller.servo_gripper(gripper_angle)
        logger.info("Successfully moved to default position")
    finally:
        controller.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="移动 Episode1 机械臂到默认位置（方便安装相机、夹爪）")
    parser.add_argument("--ip", default=DEFAULT_TCP_IP, help=f"机器人控制器的IP地址 (默认: {DEFAULT_TCP_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_TCP_PORT, help=f"机器人控制器的端口号 (默认: {DEFAULT_TCP_PORT})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TCP_TIMEOUT_S, help=f"TCP 超时秒数 (默认: {DEFAULT_TCP_TIMEOUT_S})")
    parser.add_argument("--gripper-angle", type=int, default=DEFAULT_MAINTENANCE_GRIPPER_ANGLE, help=f"归位后夹爪角度 (默认: {DEFAULT_MAINTENANCE_GRIPPER_ANGLE})")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")

    move_to_default_position(ip=args.ip, port=args.port, timeout=args.timeout, gripper_angle=args.gripper_angle)


if __name__ == "__main__":
    main()
