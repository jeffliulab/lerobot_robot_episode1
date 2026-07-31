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

"""Episode1 主/从臂共享的常量（单一出处，config 默认值与脚本都从这里取）。"""

# 速度模式。出处：恩培 fork 改 teleoperate.py/record.py 加的 --enpei_speed_mode。
# "record_forpi" 是 fork 里出现但漏进字典的模式，参数先与 record 相同，待厂商确认。
SPEED_MODES: tuple[str, ...] = ("record", "inference", "teleop", "record_forpi")

# 默认速度模式：record（最低速，最稳）。要高速必须显式指定。
DEFAULT_SPEED_MODE = "record"

# 夹爪量程（度）。出处：恩培 fork EnpeiLeader 类常量 min/max_gripper_angle = 20/110，
# 注释写明"根据自己夹爪的实际角度范围设置"。
DEFAULT_GRIPPER_MIN_ANGLE = 20.0
DEFAULT_GRIPPER_MAX_ANGLE = 110.0

# 主臂夹爪舵机的全行程角度（度），用作 leader 夹爪 → follower 夹爪映射的分母。
# 出处：恩培 fork gripper_mapping_ratio = (max-min)/90 里写死的 90。
DEFAULT_LEADER_GRIPPER_TRAVEL = 90.0

# 主臂电机 ID 范围（闭区间，1-7 表示 7 颗舵机）。出处：恩培 fork SERVO_MOTOR_RANGE。
DEFAULT_MOTOR_ID_RANGE: tuple[int, int] = (1, 7)

# 飞特总线波特率。出处：恩培 fork old_motors/feetech.py BAUDRATE = 1_000_000
# （也是 scservo_sdk PortHandler 的默认值）。
DEFAULT_BAUDRATE = 1_000_000

# 上位机（Episode1 本体控制器）TCP 默认地址。出处：恩培 fork config_enpei_follower.py。
DEFAULT_TCP_IP = "localhost"
DEFAULT_TCP_PORT = 12345
# TCP 收发超时（秒）与长度前缀上限（字节，防畸形包撑爆内存）。
DEFAULT_TCP_TIMEOUT_S = 1.0
DEFAULT_TCP_MAX_MESSAGE_BYTES = 1_048_576  # 1 MiB
# send_command 失败时的最大尝试次数（含首次）。出处：恩培 fork 写死 range(2)。
DEFAULT_TCP_MAX_ATTEMPTS = 2

# 主臂连续读失败上限：超过则抛异常走安全断开。防炸设计，默认 10。
DEFAULT_MAX_CONSECUTIVE_READ_FAILURES = 10

# 从臂首帧防跳变阈值（度）：首帧目标与 connect 时位姿差异超过此值则拒绝开始。
DEFAULT_FIRST_ACTION_MAX_JUMP_DEG = 30.0
