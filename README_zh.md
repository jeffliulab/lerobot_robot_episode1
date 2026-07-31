[![Language: English](https://img.shields.io/badge/Language-English-2f81f7?style=flat-square)](README.md) [![语言: 简体中文](https://img.shields.io/badge/语言-简体中文-e67e22?style=flat-square)](README_zh.md)

# lerobot_robot_episode1

[![lerobot](https://img.shields.io/badge/lerobot-%3E%3D0.6%2C%3C0.7-ff9d00?style=flat-square)](https://github.com/huggingface/lerobot) [![Python](https://img.shields.io/badge/Python-%3E%3D3.12-3776ab?style=flat-square)](https://www.python.org/) [![Status: Active](https://img.shields.io/badge/Status-Active-success?style=flat-square)]() [![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue?style=flat-square)](LICENSE)

> 🤖 如果你是 AI agent，先读 [AGENTS.md](AGENTS.md)。

一个 LeRobot 第三方插件，让**原版、未修改**的 [LeRobot](https://github.com/huggingface/lerobot)
直接支持 Episode1 机器人套件：Episode1 从臂（经厂商上位机 TCP 控制）和 7 舵机飞特遥操主臂。
装上本包，官方所有 CLI——`lerobot-teleoperate`、`lerobot-record`、`lerobot-train`、
`lerobot-calibrate`——都能驱动这套硬件，不再需要任何 fork。

---

## 概览

厂商的臂支持是以整个 LeRobot fork 的形式分发的，这把你钉死在一个旧快照上，还和正牌
`lerobot` 包互相冲突。本插件走的是 LeRobot 0.6 新开的另一条路：第三方设备包。发行名以
`lerobot_robot_` 开头的包会被自动发现，config 自行注册，官方工厂函数按命名约定解析
设备类——于是同一套硬件跑在上游 LeRobot 上，上游更新也照收。

## 关键特性

- **两种设备类型**：`episode1_follower`（6 关节臂 + 夹爪，TCP `localhost:12345`）和
  `episode1_leader`（7 个飞特 ST-3215 总线舵机，1 Mbps）。
- **硬件差异全在 config**：速度模式、角度/弧度、夹爪量程、电机 ID 映射都是 draccus
  配置字段（`--robot.speed_mode=record`），不打补丁。
- **安全做在设备类里**：相对 + 绝对双层动作钳制、首帧防跳变（默认 30°）、读失败用上帧
  兜底并逐级升级、幂等断开。
- **操作员工具**：`episode1-doctor`（环境/总线/相机/上位机自检）、
  `episode1-set-middle`（中位校准）、`episode1-default-position`。

## 安装

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu  # 训练时换 CUDA 版
pip install "lerobot[feetech,dataset,training]>=0.6,<0.7"
pip install -e ".[test]"
```

自检：`.venv/bin/python -m pytest tests/`（46 个无硬件测试）。

## 快速上手

```bash
episode1-doctor                     # 自检:环境、串口权限、7 舵机总线、相机、上位机
episode1-set-middle                 # 主臂中位校准(人来操作)

lerobot-teleoperate \
    --robot.type=episode1_follower --robot.ip_address=localhost --robot.port=12345 \
    --robot.speed_mode=record \
    --teleop.type=episode1_leader --teleop.port=/dev/ttyACM0 --teleop.speed_mode=record \
    --fps=30 --display_data=false
```

所有让真实硬件带电/运动的命令都应由人来执行——这台臂没有有效急停，断电就是急停。
完整规划见 `docs/ROADMAP.md`，与厂商教程命令的对照见下。

## 与厂商教程命令对照

| 教程（恩培 fork） | 本包（官方 lerobot） |
|---|---|
| `python -m lerobot.set_middle --port=/dev/ttyACM0` | `episode1-set-middle --port=/dev/ttyACM0` |
| `python -m lerobot.episode_default_position` | `episode1-default-position` |
| `--robot.type=enpei_follower --teleop.type=enpei_leader --enpei_speed_mode=record` | `--robot.type=episode1_follower --teleop.type=episode1_leader --robot.speed_mode=record --teleop.speed_mode=record` |
| `python -m lerobot.find_cameras opencv` | `lerobot-find-cameras opencv`（官方自带） |
| `python -m lerobot.test_policy`（推理） | `lerobot-rollout --policy.path=...`（官方 0.6） |

## 致谢与许可

Apache-2.0。含衍生自 [huggingface/lerobot](https://github.com/huggingface/lerobot) 与
[enpeizhao/lerobot_single_student](https://github.com/enpeizhao/lerobot_single_student)
（均为 Apache-2.0）的代码——见 [NOTICE](NOTICE)。
