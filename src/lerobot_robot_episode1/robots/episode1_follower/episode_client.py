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
# (fork 里叫 episode_server.py,但它明明是 TCP 客户端,改名 episode_client.py;
#  防炸加固:长度前缀上限/JSON 校验/收发超时/重连上限;协议错误=断开报错,不再静默返回 None)

import json
import logging
import socket

import numpy as np

from ...constants import (
    DEFAULT_TCP_IP,
    DEFAULT_TCP_MAX_ATTEMPTS,
    DEFAULT_TCP_MAX_MESSAGE_BYTES,
    DEFAULT_TCP_PORT,
    DEFAULT_TCP_TIMEOUT_S,
)

logger = logging.getLogger(__name__)

# 长度前缀字节数（8 字节大端无符号整数）。协议约定，来自恩培上位机。
_LEN_PREFIX_BYTES = 8


class EpisodeClientError(ConnectionError):
    """TCP 客户端错误：连接失败、超时、重连耗尽。"""


class EpisodeProtocolError(EpisodeClientError):
    """协议错误：长度前缀超限、响应不是合法 JSON。协议错误 = 断开报错，不猜测执行。"""


class EpisodeClient:
    """Episode1 上位机 TCP 客户端（fork 里的 EpisodeAPP，改名以免误导——它是客户端）。"""

    def __init__(
        self,
        ip: str = DEFAULT_TCP_IP,
        port: int = DEFAULT_TCP_PORT,
        timeout: float = DEFAULT_TCP_TIMEOUT_S,
        max_message_bytes: int = DEFAULT_TCP_MAX_MESSAGE_BYTES,
        max_attempts: int = DEFAULT_TCP_MAX_ATTEMPTS,
    ):
        """
        初始化客户端，设置服务器的IP地址和端口号。
        使用持久连接，避免每次命令都创建新的TCP连接。

        参数：
            timeout (float): 收发超时（秒）。
            max_message_bytes (int): 长度前缀上限（字节），防畸形包撑爆内存。
            max_attempts (int): 单条命令的最大尝试次数（含首次），连接断开时自动重连。
        """
        self.server_address = (ip, port)
        self.timeout = timeout
        self.max_message_bytes = max_message_bytes
        self.max_attempts = max_attempts
        self._socket = None

    def _ensure_connected(self):
        """确保持久连接可用，如果断开则重新连接。"""
        if self._socket is None:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._socket.settimeout(self.timeout)
            self._socket.connect(self.server_address)
            logger.info(f"已建立到 {self.server_address} 的持久连接")

    def close(self):
        """关闭持久连接（幂等）。"""
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

    # fork 里 follower 直接调 _close_connection，保留别名兼容
    _close_connection = close

    def send_command(self, command: dict):
        """
        发送命令到服务器并接收返回值。
        使用持久TCP连接，连接断开时自动重连，最多尝试 max_attempts 次。

        参数：
            command (dict): 包含动作和参数的命令字典。

        返回：
            从服务器接收到的结果（反序列化后的对象）。

        抛出：
            EpisodeClientError: 连接/超时且重连耗尽。
            EpisodeProtocolError: 协议违规（长度超限、非法 JSON）。
        """
        last_error: Exception | None = None
        for attempt in range(self.max_attempts):
            try:
                self._ensure_connected()

                data = json.dumps(command).encode("utf-8")
                if len(data) > self.max_message_bytes:
                    raise EpisodeProtocolError(
                        f"待发送命令 {len(data)} 字节超过上限 {self.max_message_bytes}，拒绝发送"
                    )

                self._socket.sendall(len(data).to_bytes(_LEN_PREFIX_BYTES, byteorder="big"))
                self._socket.sendall(data)

                response_length = self._recv_exact(_LEN_PREFIX_BYTES)
                if response_length is None:
                    raise EpisodeClientError("服务器未返回响应长度（连接中断）")
                response_length = int.from_bytes(response_length, byteorder="big")
                if response_length > self.max_message_bytes:
                    raise EpisodeProtocolError(
                        f"服务器声明的响应长度 {response_length} 字节超过上限 "
                        f"{self.max_message_bytes}，疑似畸形包，断开连接"
                    )

                response_data = self._recv_exact(response_length)
                if response_data is None:
                    raise EpisodeClientError("服务器未返回完整响应数据（连接中断）")

                try:
                    return json.loads(response_data.decode("utf-8"))
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    raise EpisodeProtocolError(f"响应不是合法 JSON: {e}") from e
            except EpisodeProtocolError:
                # 协议错误 = 断开报错，不重试、不猜测执行
                self.close()
                raise
            except (OSError, EpisodeClientError) as e:
                last_error = e
                logger.error(f"send_command 出错 (尝试 {attempt + 1}/{self.max_attempts}): {e}")
                self.close()

        raise EpisodeClientError(
            f"send_command 在 {self.max_attempts} 次尝试后仍失败: {last_error}"
        ) from last_error

    def _recv_exact(self, num_bytes: int) -> bytes | None:
        """接收指定字节数的数据。对端正常关闭返回 None；超时抛 socket.timeout。"""
        data = b""
        while len(data) < num_bytes:
            packet = self._socket.recv(num_bytes - len(data))
            if not packet:
                return None
            data += packet
        return data

    def emergency_stop(self, enable):
        """
        急停或解除急停操作。

        参数：
            enable (int): 1 表示急停，0 表示解除急停。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {"action": "emergency_stop", "params": enable}
        return self.send_command(command)

    def angle_mode(self, angles, speed_ratio=1):
        """
        通过角度模式控制电机运动，移动到指定关节角度。

        参数：
            angles (list): 各个电机目标角度列表。
            speed_ratio (float): 运动速度比例，默认为 1。

        返回：
            服务器返回的预计运动时间（秒）。
        """
        command = {"action": "angle_mode", "params": [angles, speed_ratio]}
        return self.send_command(command)

    def sync_motor_angles(self):
        """
        同步电机角度
        """
        command = {"action": "sync_motor_angles", "params": []}
        return self.send_command(command)

    def move_xyz_rotation(self, position, orientation, rotation_order="zyx", speed_ratio=1):
        """
        通过 XYZ 坐标和欧拉角控制电机运动，采用普通位姿模式。

        参数：
            position (list): 三维空间位置，包含 [x, y, z]。
            orientation (list): 欧拉角，包含 [roll, pitch, yaw]。
            rotation_order (str): 旋转顺序（支持 "zyx" 或 "xyz"，默认 "zyx"）。
            speed_ratio (float): 运动速度比例（0~1之间，默认 1，超过该范围会自动截断）。

        返回值：
            -1 表示 IK 无解；
            >0 表示有解，返回预计运动时间（秒），建议客户端等待该时间后再发送下一条指令。
        """
        params = position + orientation + [rotation_order, speed_ratio]
        command = {"action": "move_xyz_rotation", "params": params}
        return self.send_command(command)

    def move_linear_xyz_rotation(self, position, orientation, rotation_order="zyx"):
        """
        采用直线模式，通过 XYZ 坐标和欧拉角控制电机运动。直线模式下无法调整速度，
        运动过程中服务器会先返回运动时间再执行运动操作（可能会阻塞）。

        参数：
            position (list): 三维空间位置，包含 [x, y, z]。
            orientation (list): 欧拉角，包含 [roll, pitch, yaw]。
            rotation_order (str): 旋转顺序（支持 "zyx" 或 "xyz"，默认 "zyx"）。

        返回值：
            -1 表示 IK 求解无解；
            >0 表示有解，返回预计运动时间（秒）。
        """
        params = position + orientation + [rotation_order]
        command = {"action": "move_linear_xyz_rotation", "params": params}
        return self.send_command(command)

    def gripper_on(self):
        """
        启动负压吸盘抓取操作。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {"action": "gripper_on"}
        return self.send_command(command)

    def gripper_off(self):
        """
        负压吸盘释放操作。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {"action": "gripper_off"}
        return self.send_command(command)

    def servo_gripper(self, angle):
        """
        通过舵机控制夹爪角度。

        参数：
            angle (int): 夹爪角度。

        返回：
            固定返回 1（单位：秒）。
        """
        command = {"action": "servo_gripper", "params": angle}
        return self.send_command(command)

    def robodk_simu(self, enable):
        """
        控制 Robodk 模拟开关。

        参数：
            enable (int): 1 表示开启模拟，0 表示关闭模拟。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {"action": "robodk_simu", "params": enable}
        return self.send_command(command)

    def set_free_mode(self, mode):
        """
        设置电机自由模式。注意：在进入自由模式前需要提醒用户准备好托举。

        参数：
            mode (int): 1 进入自由模式，0 退出自由模式。

        返回：
            固定返回 0.1（单位：秒）。
        """
        command = {"action": "set_free_mode", "params": mode}
        return self.send_command(command)

    def get_motor_angles(self):
        """
        获取当前电机的角度信息。

        返回：
            一个长度为 6 的角度列表（单位：度）。由于 CAN 总线阻塞，部分电机角度读取可能失败，此时返回 None。
        """
        command = {"action": "get_motor_angles"}
        return self.send_command(command)

    def dynamic_move(self, goal_pos, current_joint_pulses, motors_max_speed):
        """
        动态移动到目标位置。

        参数：
            goal_pos (list): 目标位置，包含 6 个关节角度。
            current_joint_pulses (list): 当前关节脉冲数，包含 6 个值。
            motors_max_speed (list): 电机最大速度，包含 6 个值。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {"action": "dynamic_move", "params": [goal_pos, current_joint_pulses, motors_max_speed]}
        return self.send_command(command)

    def get_T(self):
        """
        获取齐次变换矩阵 T。
        """
        command = {"action": "get_T"}
        result = self.send_command(command)
        if result is None:
            return None
        # 将列表转换为 numpy 数组并返回
        return np.array(result).reshape(4, 4)

    def get_pose(self, rotation_order="xyz"):
        """
        获取位姿，支持xyz和zyx两种旋转顺序。
        """
        command = {"action": "get_pose", "params": rotation_order}
        result = self.send_command(command)
        if result is None:
            return None
        return np.array(result)


# 兼容旧名（fork 里的 EpisodeAPP）
EpisodeAPP = EpisodeClient
