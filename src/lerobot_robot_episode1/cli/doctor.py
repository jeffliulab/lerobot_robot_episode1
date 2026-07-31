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
# (doctor 是新写的自检命令,fork 里没有)

"""episode1-doctor：装机/排障自检。逐项检查并给出"怎么修"，无硬件环境也输出清晰报告。"""

import argparse
import glob
import grp
import importlib
import importlib.metadata
import os
import socket
import sys

from ..constants import (
    DEFAULT_BAUDRATE,
    DEFAULT_MOTOR_ID_RANGE,
    DEFAULT_TCP_IP,
    DEFAULT_TCP_PORT,
    DEFAULT_TCP_TIMEOUT_S,
)

# lerobot 插件机制自 0.6.0 起可用，pyproject 也锁了 >=0.6,<0.7
_LEROBOT_VERSION_RANGE = ((0, 6, 0), (0, 7, 0))  # [min, max)


def _parse_version(v: str) -> tuple[int, ...]:
    parts = []
    for piece in v.split("."):
        num = ""
        for ch in piece:
            if ch.isdigit():
                num += ch
            else:
                break
        parts.append(int(num) if num else 0)
    return tuple(parts)


def check_lerobot_version() -> tuple[bool, str, str]:
    try:
        version = importlib.metadata.version("lerobot")
    except importlib.metadata.PackageNotFoundError:
        return False, "未安装 lerobot", "pip install 'lerobot[feetech]>=0.6,<0.7'"
    (lo, hi) = _LEROBOT_VERSION_RANGE
    parsed = _parse_version(version)
    ok = tuple(lo) <= (parsed + (0, 0, 0))[: 3] < tuple(hi)
    if ok:
        return True, f"lerobot {version}", ""
    return False, f"lerobot {version} 不在 [0.6, 0.7) 区间", "pip install 'lerobot[feetech]>=0.6,<0.7'"


def check_scservo_sdk() -> tuple[bool, str, str]:
    try:
        importlib.import_module("scservo_sdk")
        return True, "scservo_sdk 可导入", ""
    except ImportError as e:
        return False, f"scservo_sdk 导入失败: {e}", "pip install feetech-servo-sdk"


def check_serial_port(port: str) -> tuple[bool, str, str]:
    if not os.path.exists(port):
        available = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        hint = f"当前可用串口: {available}" if available else "没有发现任何 /dev/ttyACM* 或 /dev/ttyUSB*，请检查 USB 线"
        return False, f"串口 {port} 不存在", hint
    if not os.access(port, os.R_OK | os.W_OK):
        in_dialout = any(grp.getgrgid(g).gr_name == "dialout" for g in os.getgroups())
        hint = "权限不足。" + (
            "已在 dialout 组但仍无权限，请检查 udev 规则"
            if in_dialout
            else "执行 `sudo usermod -aG dialout $USER` 后重新登录"
        )
        return False, f"串口 {port} 存在但不可读写", hint
    return True, f"串口 {port} 存在且可读写", ""


def check_bus_scan(port: str, motor_id_range: tuple[int, int], motor_model: str) -> tuple[bool, str, str]:
    """只读 ping：期望总线上恰好每颗 ID 各一颗。"""
    if not os.path.exists(port):
        return False, "跳过（串口不存在）", "先解决串口问题"
    try:
        from ..motors.servo_controller import FeetechController

        controller = FeetechController(port=port, motor_model=motor_model, motor_range=motor_id_range)
        controller.connect()
        try:
            bad_ids = []
            for motor_id in range(motor_id_range[0], motor_id_range[1] + 1):
                try:
                    controller.motors_bus.read_with_motor_ids([motor_model], [motor_id], "ID", num_retry=2)
                except Exception:
                    bad_ids.append(motor_id)
        finally:
            controller.disconnect()
        if bad_ids:
            return (
                False,
                f"ID {bad_ids} 无响应",
                "检查是否漏插/串联线断开/两颗电机 ID 重复（冲突同样表现为读取失败）",
            )
        return True, f"总线上 ID {motor_id_range[0]}-{motor_id_range[1]} 全部响应", ""
    except Exception as e:
        return False, f"总线扫描失败: {e}", "检查串口是否被占用、电机供电与波特率"


def check_cameras() -> tuple[bool, str, str]:
    videos = sorted(glob.glob("/dev/video*"))
    if not videos:
        return False, "没有发现 /dev/video*", "检查相机 USB 连接（不用相机可忽略）"
    return True, f"发现 {len(videos)} 个视频设备: {videos}", ""


def check_tcp(ip: str, port: int, timeout: float) -> tuple[bool, str, str]:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True, f"上位机 {ip}:{port} TCP 可达", ""
    except OSError as e:
        return False, f"上位机 {ip}:{port} TCP 不可达: {e}", "确认 Episode1 上位机已开机、IP/端口正确、网络互通"


def main() -> None:
    parser = argparse.ArgumentParser(description="Episode1 装机/排障自检")
    parser.add_argument("--port", default="/dev/ttyACM0", help="主臂串口 (默认: /dev/ttyACM0)")
    parser.add_argument("--ip", default=DEFAULT_TCP_IP, help=f"上位机 IP (默认: {DEFAULT_TCP_IP})")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT, help=f"上位机 TCP 端口 (默认: {DEFAULT_TCP_PORT})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TCP_TIMEOUT_S, help=f"TCP 探测超时秒数 (默认: {DEFAULT_TCP_TIMEOUT_S})")
    parser.add_argument("--motor-range", type=int, nargs=2, default=list(DEFAULT_MOTOR_ID_RANGE), metavar=("START", "END"), help="主臂电机 ID 范围 (默认: 1 7)")
    parser.add_argument("--skip-bus-scan", action="store_true", help="跳过总线扫描（不接主臂时）")
    args = parser.parse_args()

    checks: list[tuple[str, tuple[bool, str, str]]] = []
    checks.append(("lerobot 版本", check_lerobot_version()))
    checks.append(("scservo_sdk", check_scservo_sdk()))
    checks.append(("串口", check_serial_port(args.port)))
    if args.skip_bus_scan:
        checks.append(("总线扫描", (True, "已按 --skip-bus-scan 跳过", "")))
    else:
        checks.append(("总线扫描", check_bus_scan(args.port, tuple(args.motor_range), "sts3215")))
    checks.append(("相机", check_cameras()))
    checks.append(("上位机 TCP", check_tcp(args.ip, args.tcp_port, args.timeout)))

    print("\n===== episode1-doctor 自检报告 =====")
    all_ok = True
    for name, (ok, message, hint) in checks:
        mark = "✓" if ok else "✗"
        print(f"{mark} {name}: {message}")
        if not ok:
            all_ok = False
            if hint:
                print(f"    修复提示: {hint}")
    print("===================================")
    print("全部通过，可以进行下一步。" if all_ok else "存在未通过项，请按上面的提示修复后重跑 episode1-doctor。")

    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
