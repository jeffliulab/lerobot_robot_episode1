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
# (fork 的 src/lerobot/set_middle.py 改造而来:draccus 换成 argparse,
#  校准结果改按官方 calibration_dir 机制存放)

# 执行中位位置校准
import argparse
import sys
import json
import logging
import time

from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from ..constants import DEFAULT_MOTOR_ID_RANGE
from ..motors.servo_controller import FeetechController

logger = logging.getLogger(__name__)


# Helper functions for non-blocking keyboard input
def input_available():
    """Check if keyboard input is available"""
    try:
        import sys, select
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    except Exception:
        # Fallback method if select doesn't work
        return False


def get_key():
    """Get a single keypress（调用方已把终端切成 cbreak，这里直接读一个字符）"""
    import sys
    return sys.stdin.read(1)



def reset_middle_positions(controller, motor_range):
    """
    重置中位，将所有电机设置为中位位置
    类似于 lerobot 中的 reset_middle_positions 函数

    Args:
        controller: 电机控制器
        motor_range: 电机ID范围
    """
    print("请手动移动机械臂到新的中位位置，然后按回车...")
    input()

    # 写入 128 到所有电机的 Torque_Enable
    motor_names = [f"motor{i}" for i in range(motor_range[0], motor_range[1] + 1)]
    controller.motors_bus.write("Torque_Enable", 128, motor_names)
    time.sleep(0.5)
    after = controller.batch_read_positions()
    print("\n已重置所有电机中位位置。写入后读数:", ", ".join(f"电机{i}: {int(v)}" for i, v in after.items()))


def read_raw_position(controller, motor_id):
    """
    读取指定电机的原始位置值（不转换为角度）

    Args:
        controller: 电机控制器
        motor_id: 电机ID

    Returns:
        原始位置值
    """
    motor_name = f"motor{motor_id}"
    position = controller.motors_bus.read("Present_Position", motor_name)
    return position


def toggle_motor_lock(controller, motor_range, motors_locked):
    """切换电机锁定状态（启用/禁用扭矩）

    Args:
        controller: 电机控制器
        motor_range: 电机ID范围
        motors_locked: 当前电机锁定状态

    Returns:
        新的电机锁定状态
    """
    # 切换状态
    motors_locked = not motors_locked

    # 应用扭矩设置
    torque_value = 1 if motors_locked else 0  # 1 = TorqueMode.ENABLED, 0 = TorqueMode.DISABLED

    # 为所有电机设置扭矩
    motor_names = [f"motor{i}" for i in range(motor_range[0], motor_range[1] + 1)]
    controller.motors_bus.write("Torque_Enable", torque_value, motor_names)

    # 打印状态信息
    status = "已锁定 (扭矩启用)" if motors_locked else "已解锁 (扭矩禁用)"
    print(f"\n电机 {status}")

    return motors_locked


def save_zero_references(controller, motor_range, calib_id: str) -> None:
    """把当前各电机的原始读数存为零点参考，按官方 calibration_dir 机制存放。"""
    positions = controller.batch_read_positions()
    calib_dir = HF_LEROBOT_CALIBRATION / TELEOPERATORS / "episode1_leader"
    calib_dir.mkdir(parents=True, exist_ok=True)
    fpath = calib_dir / f"{calib_id}.json"
    with open(fpath, "w") as f:
        json.dump({str(k): int(v) for k, v in positions.items()}, f, indent=4)
    print(f"零点参考已保存到 {fpath}")


MIDDLE_TARGET = 2048          # 飞特舵机一圈 4096 格的正中
MIDDLE_TOLERANCE = 100        # 校准后读数离 2048 超过这个值就当没写进去，提示重试


def set_middle_now(controller, motor_range, calib_id: str) -> bool:
    """不进交互循环：读一次 → 写中位（Torque_Enable=128）→ 再读一次给人对比 → 存零点参考。

    适合终端收不到按键（输入法、select 在行缓冲下要等回车）的情况。
    返回 True 表示所有电机读数都落在 2048±MIDDLE_TOLERANCE。
    """
    motor_names = [f"motor{i}" for i in range(motor_range[0], motor_range[1] + 1)]
    controller.motors_bus.write("Torque_Enable", 0, motor_names)
    before = controller.batch_read_positions()
    print("写入前原始读数:", ", ".join(f"电机{i}: {int(v)}" for i, v in before.items()))
    controller.motors_bus.write("Torque_Enable", 128, motor_names)   # 飞特「中位校准」：当前位置定义为 2048
    time.sleep(0.5)
    after = controller.batch_read_positions()
    print("写入后原始读数:", ", ".join(f"电机{i}: {int(v)}" for i, v in after.items()))
    bad = {i: int(v) for i, v in after.items() if abs(int(v) - MIDDLE_TARGET) > MIDDLE_TOLERANCE}
    if bad:
        print(f"⚠️ 这些电机没落到 {MIDDLE_TARGET}±{MIDDLE_TOLERANCE}: {bad}。"
              "检查该电机通电/接线，或把关节稍微动一下后重跑本命令。未保存零点参考。")
        return False
    print("✅ 全部电机已定义为中位 2048")
    save_zero_references(controller, motor_range, calib_id)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Episode1 主臂中位（零点）校准")
    parser.add_argument("--port", default="/dev/ttyACM0", help="串口设备路径 (默认: /dev/ttyACM0)")
    parser.add_argument(
        "--motor-range", type=int, nargs=2, default=list(DEFAULT_MOTOR_ID_RANGE),
        metavar=("START", "END"), help="电机ID范围，闭区间 (默认: 1 7)",
    )
    parser.add_argument(
        "--id", default="default",
        help="校准文件 id（与 --teleop.id 对应，默认: default）。退出时可选择把当前读数存为零点参考。",
    )
    parser.add_argument(
        "--now", action="store_true",
        help="不进交互循环：把主臂当前姿态直接定义为中位并保存零点参考（先手动把主臂摆好再跑）",
    )
    args = parser.parse_args()
    motor_range = tuple(args.motor_range)

    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")

    # 初始化控制器
    controller = FeetechController(port=args.port, motor_range=motor_range)

    # 连接到电机
    controller.connect()
    if args.now:
        try:
            ok = set_middle_now(controller, motor_range, args.id)
        finally:
            controller.disconnect()
            print("控制器已断开连接。")
        raise SystemExit(0 if ok else 1)

    # 启动时先解锁所有电机
    motor_names = [f"motor{i}" for i in range(motor_range[0], motor_range[1] + 1)]
    controller.motors_bus.write("Torque_Enable", 0, motor_names)
    print("已解锁所有电机（扭矩禁用）")

    # 电机锁定状态（True = 锁定/扭矩启用，False = 解锁/扭矩禁用）
    motors_locked = False

    # 把终端切到 cbreak：单个按键立即可读，不用回车（Ctrl+C 仍然有效）
    _fd = None; _old_tty = None
    try:
        import termios, tty
        _fd = sys.stdin.fileno(); _old_tty = termios.tcgetattr(_fd); tty.setcbreak(_fd)
    except Exception:
        pass  # 非终端（如管道）就退回原来的 select 探测
    try:
        print(f"开始持续监控位置。电机范围: {motor_range[0]}-{motor_range[1]}。按 Ctrl+C 停止。")
        print("您可以在监控时手动移动机械臂。")
        print("按 'r' 重置中位位置。")
        print("按 'l' 切换电机锁定状态（启用/禁用扭矩）。")

        while True:
            # 读取所有电机的当前位置
            start_time = time.perf_counter()
            positions = {}
            for motor_id in range(motor_range[0], motor_range[1] + 1):
                positions[motor_id] = read_raw_position(controller, motor_id)

            # 显示原始位置
            lock_status = "🔒" if motors_locked else "🔓"
            positions_str = ", ".join([f"电机 {i}: {positions[i]}" for i in range(motor_range[0], motor_range[1] + 1)])

            # 检查键盘输入
            if input_available():
                key = get_key()
                if key.lower() == 'r':
                    # 重置中位位置
                    reset_middle_positions(controller, motor_range)
                elif key.lower() == 'l':
                    # 切换电机锁定状态
                    motors_locked = toggle_motor_lock(controller, motor_range, motors_locked)

            # 计算FPS
            end_time = time.perf_counter()
            fps = 1 / (end_time - start_time)
            print(f"原始位置: {positions_str} {lock_status} | FPS: {fps:.2f}", end="\r")

    except KeyboardInterrupt:
        print("\n用户停止监控。")
    finally:
        if _old_tty is not None:
            import termios
            termios.tcsetattr(_fd, termios.TCSADRAIN, _old_tty)
        # 确保在断开连接前解锁电机（禁用扭矩）
        if motors_locked:
            motor_names = [f"motor{i}" for i in range(motor_range[0], motor_range[1] + 1)]
            controller.motors_bus.write("Torque_Enable", 0, motor_names)
            print("电机已解锁（扭矩禁用）")

        # 可选：把当前读数存为该臂的零点参考（官方 calibration_dir 机制）
        try:
            answer = input(f"是否把当前各电机读数保存为 id={args.id} 的零点参考？[y/N] ")
            if answer.strip().lower() == "y":
                save_zero_references(controller, motor_range, args.id)
        except (EOFError, KeyboardInterrupt):
            pass

        # 完成后断开连接
        controller.disconnect()
        print("控制器已断开连接。")


if __name__ == "__main__":
    main()
