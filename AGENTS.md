# AGENTS.md · lerobot_robot_episode1

## 是什么 / 不是什么

Episode1 机器人套件的 **LeRobot 第三方插件**：向官方 lerobot（PyPI ≥0.6，未修改）注册
`episode1_follower`（从臂，走上位机 TCP API）与 `episode1_leader`（7 舵机飞特遥操主臂，串口总线）。
**不是** lerobot 的 fork，**不是** 完整机器人栈——训练、数据集、相机等全部用官方 lerobot 的能力。

代码渊源：从 `enpeizhao/lerobot_single_student`（Apache-2.0）移植并适配官方插件体系，
基点为官方 lerobot `519b7611`（2025-07-13）。保留版权头 + 注明修改，见 NOTICE。

## 任务 → 去哪查

| 想改什么 | 去哪 |
|---|---|
| 从臂行为（TCP 协议、钳制、首帧防跳变） | `src/lerobot_robot_episode1/robots/episode1_follower/` |
| 主臂行为（舵机读取、滤波、读失败策略） | `src/lerobot_robot_episode1/teleoperators/episode1_leader/` |
| 飞特总线驱动（ vendored 老版，自包含） | `src/lerobot_robot_episode1/motors/` |
| 默认值（速度表/滤波/夹爪量程/ID 范围/波特率） | 各 `config_*.py` 的 config 字段，**不许散落到逻辑代码里** |
| 自检 / 中位校准 / 默认位 | `src/lerobot_robot_episode1/cli/` |
| 硬件背景与舵机编号事实 | `~/episode-robot-dev-framework/episode-leader-arm/README.md` |

## 目录地图

- `src/lerobot_robot_episode1/robots/episode1_follower/` — 从臂 Robot 类 + TCP 客户端（episode_client.py）
- `src/lerobot_robot_episode1/teleoperators/episode1_leader/` — 主臂 Teleoperator 类
- `src/lerobot_robot_episode1/motors/` — 老版 feetech 总线驱动 + 薄封装（只依赖 scservo_sdk）
- `src/lerobot_robot_episode1/cli/` — `episode1-doctor` / `episode1-set-middle` / `episode1-default-position`
- `src/lerobot_robot_episode1/data/` — 参考零点 json（只作参考，正式校准走官方 calibration_dir）
- `scripts/` — 一次性数据集迁移工具（不装 entry point）
- `tests/` — 无硬件测试（fake 串口 / mock TCP）

## 怎么跑起来

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu   # 训练再换 CUDA 版
pip install "lerobot[feetech]>=0.6,<0.7"
pip install -e ".[test]"
.venv/bin/python -m pytest tests/      # 自检：全绿
episode1-doctor                        # 硬件/环境自检（真机项由人跑）
```

遥操 / 校准 / 教程命令对照：见 README。

## 红线

- ⛔ **不改官方 lerobot 源码**。设备差异只能经 config 字段与插件注册进入。
- ⛔ **不碰真机**：凡会连接/上电/驱动真实硬件的命令（串口、上位机 TCP）由用户亲手执行；
  agent 只写代码、跑无硬件测试。
- ⛔ **禁硬编码**：速度表、量程、ID 范围、波特率、超时等一律 config 字段化，默认值带出处注释。
- ⛔ 主臂读失败不得返回 0/None 充当动作；从臂动作必须先过钳制；disconnect 必须幂等不抛异常。
- vendored 老驱动（motors/）除非修 bug 否则不动——它是历史快照，不是发展新功能的地方。

## 提交约定

- 不写 `Co-Authored-By`；禁 `git add -A`（逐个加文件）；push 由作者发话。

## 备注

若本地同目录存在 `CLAUDE.md`，请一并阅读（内部开发笔记，未入库）。
