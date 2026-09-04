# Copyright 2026 Jeff. Licensed under the Apache License, Version 2.0.
"""episode1-pose-check：遥操启动前的只读对姿工具。

连主臂、把它当前姿态映射成从臂目标角，逐关节和「从臂启动后会自己去的位姿」
（Episode1FollowerConfig.startup_joint_positions_2）比，差 >30° 标红，每 0.5s 刷新。
全绿再去跑 lerobot-teleoperate，就不会被首帧保护拦下。

⛔ 只读：不给主臂上力矩，不连从臂的 Robot 类（那会让从臂动），只用 TCP 读一下从臂当前角度作参考。
"""
import argparse
import sys
import time

from ..constants import DEFAULT_TCP_IP, DEFAULT_TCP_PORT, DEFAULT_TCP_TIMEOUT_S
from ..robots.episode1_follower.config_episode1_follower import Episode1FollowerConfig
from ..teleoperators.episode1_leader.config_episode1_leader import Episode1LeaderConfig
from ..teleoperators.episode1_leader.episode1_leader import Episode1Leader

JOINTS = [f"joint{i}" for i in range(1, 7)]


def compare(action: dict[str, float], startup_deg: list[float], max_jump_deg: float) -> list[tuple[str, float, float, float, bool]]:
    """逐关节：(关节, 主臂映射目标, 从臂启动位姿, 差, 是否在阈值内)。夹爪不参与首帧保护，不比。"""
    rows = []
    for i, j in enumerate(JOINTS):
        goal = float(action[f"{j}.pos"])
        ref = float(startup_deg[i])
        diff = abs(goal - ref)
        rows.append((j, goal, ref, diff, diff <= max_jump_deg))
    return rows


def render(rows, gripper: float | None, follower_now: list[float] | None) -> str:
    lines = ["关节    主臂映射目标   从臂启动位姿     差      判定" + ("     从臂此刻" if follower_now else "")]
    for k, (j, goal, ref, diff, ok) in enumerate(rows):
        tail = f"   {follower_now[k]:7.1f}°" if follower_now else ""
        lines.append(f"{j:8s} {goal:8.1f}°     {ref:7.1f}°   {diff:6.1f}°   {'✅' if ok else '⛔ 调这个'}{tail}")
    if gripper is not None:
        lines.append(f"gripper  {gripper:8.1f}°     （夹爪不参与首帧保护）")
    lines.append("全部 ✅ 就可以启动 lerobot-teleoperate。Ctrl+C 退出。" if all(r[4] for r in rows)
                 else "⛔ 把标红的关节朝「从臂启动位姿」那个数的方向转，直到差 ≤ 阈值。Ctrl+C 退出。")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="遥操启动前只读对姿：主臂映射角 vs 从臂启动位姿")
    parser.add_argument("--port", default="/dev/ttyACM0", help="主臂串口 (默认: /dev/ttyACM0)")
    parser.add_argument("--id", default="episode1_leader", help="主臂校准 id（与 --teleop.id 相同，默认: episode1_leader）")
    parser.add_argument("--ip", default=DEFAULT_TCP_IP, help="上位机 IP，用来只读从臂此刻角度（默认: localhost）")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT, help="上位机 TCP 端口 (默认: 12345)")
    parser.add_argument("--no-follower", action="store_true", help="不连上位机，只看主臂映射")
    parser.add_argument("--interval", type=float, default=0.5, help="刷新间隔秒 (默认: 0.5)")
    args = parser.parse_args()

    fcfg = Episode1FollowerConfig()
    startup = list(fcfg.startup_joint_positions_2)[:6]
    threshold = fcfg.first_action_max_jump_deg

    leader = Episode1Leader(Episode1LeaderConfig(port=args.port, id=args.id))
    leader.connect(calibrate=False)

    client = None
    if not args.no_follower:
        try:
            from ..robots.episode1_follower.episode_client import EpisodeClient
            client = EpisodeClient(ip=args.ip, port=args.tcp_port, timeout=DEFAULT_TCP_TIMEOUT_S)
        except Exception as e:  # 上位机没开也能用，只是少一列
            print(f"（未连上上位机，只看主臂映射：{e}）")
            client = None

    print(f"从臂启动后会自己去 {startup}（+夹爪 {fcfg.startup_gripper_angle}°）；首帧保护阈值 {threshold}°。\n")
    try:
        while True:
            action = leader.get_action()
            rows = compare(action, startup, threshold)
            now = None
            if client is not None:
                try:
                    r = client.get_motor_angles()
                    now = [float(x) for x in r] if isinstance(r, (list, tuple)) and len(r) >= 6 else None
                except Exception:
                    now = None
            sys.stdout.write("\033[2J\033[H" + render(rows, action.get("gripper.pos"), now) + "\n")
            sys.stdout.flush()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n退出对姿。")
    finally:
        try:
            leader.disconnect()
        except Exception:
            pass
        if client is not None:
            try:
                client.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
