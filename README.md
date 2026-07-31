[![Language: English](https://img.shields.io/badge/Language-English-2f81f7?style=flat-square)](README.md) [![语言: 简体中文](https://img.shields.io/badge/语言-简体中文-e67e22?style=flat-square)](README_zh.md)

# lerobot_robot_episode1

[![lerobot](https://img.shields.io/badge/lerobot-%3E%3D0.6%2C%3C0.7-ff9d00?style=flat-square)](https://github.com/huggingface/lerobot) [![Python](https://img.shields.io/badge/Python-%3E%3D3.12-3776ab?style=flat-square)](https://www.python.org/) [![Status: Active](https://img.shields.io/badge/Status-Active-success?style=flat-square)]() [![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue?style=flat-square)](LICENSE)

> 🤖 If you are an AI agent, read [AGENTS.md](AGENTS.md) first.

A third-party [LeRobot](https://github.com/huggingface/lerobot) plugin that teaches **stock,
unmodified** LeRobot to drive the Episode1 robot kit: the Episode1 follower arm (controlled
through its vendor TCP server) and the 7-servo Feetech teleoperation leader arm. Install the
package and every official LeRobot CLI — `lerobot-teleoperate`, `lerobot-record`,
`lerobot-train`, `lerobot-calibrate` — works with this hardware, no fork required.

---

## Overview

The vendor ships their arm support as a full fork of LeRobot, which pins you to an old
snapshot and collides with the real `lerobot` package. This plugin takes the other road
LeRobot opened in 0.6: third-party device packages. Distribution names starting with
`lerobot_robot_` are auto-discovered, configs register themselves, and the official factories
resolve device classes by convention — so the same hardware works on upstream LeRobot, and
upstream updates keep flowing in.

## Key features

- **Two device types**: `episode1_follower` (6-joint arm + gripper over TCP
  `localhost:12345`) and `episode1_leader` (7 Feetech ST-3215 bus servos at 1 Mbps).
- **Hardware quirks as config**: speed modes, degree/radian units, gripper range, motor ID
  map are all draccus config fields (`--robot.speed_mode=record`), not patches to LeRobot.
- **Safety inside the device classes**: relative + absolute action clamping, first-frame
  jump guard (30° by default), stale-read fallback with failure escalation, idempotent
  disconnect.
- **Operator tooling**: `episode1-doctor` (environment/bus/camera/server self-check),
  `episode1-set-middle` (mid-position calibration), `episode1-default-position`.

## Installation

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu  # swap for CUDA build when training
pip install "lerobot[feetech,dataset,training]>=0.6,<0.7"
pip install -e ".[test]"
```

Sanity check: `.venv/bin/python -m pytest tests/` (46 hardware-free tests).

## Quick start

```bash
episode1-doctor                     # self-check: env, serial perms, 7-servo bus, cameras, TCP server
episode1-set-middle                 # calibrate the leader arm (human-operated)

lerobot-teleoperate \
    --robot.type=episode1_follower --robot.ip_address=localhost --robot.port=12345 \
    --robot.speed_mode=record \
    --teleop.type=episode1_leader --teleop.port=/dev/ttyACM0 --teleop.speed_mode=record \
    --fps=30 --display_data=false
```

Every command that powers or moves real hardware is meant to be run by a human operator —
the arm has no effective e-stop; cutting power is the stop. See `docs/ROADMAP.md` for the
full plan and `README`'s Chinese twin for the vendor-tutorial command mapping.

## Credits & license

Apache-2.0. Contains code derived from [huggingface/lerobot](https://github.com/huggingface/lerobot)
and [enpeizhao/lerobot_single_student](https://github.com/enpeizhao/lerobot_single_student)
(both Apache-2.0) — see [NOTICE](NOTICE).
