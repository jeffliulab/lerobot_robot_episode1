"""端到端：官方 lerobot teleop_loop + 本包两个设备类（全部 fake 硬件，无真机）。

验证插件与官方 0.6 控制循环的兼容性：主臂读舵机 → 动作经官方管线 → 从臂钳制下发，
跑若干帧后干净断开。这条链路就是真机遥操的软件路径，只是硬件全换成 fake。
"""

import lerobot_robot_episode1.robots.episode1_follower.episode1_follower as follower_mod
from conftest import FakeEpisodeController
from lerobot.processor.factory import make_default_processors
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from lerobot_robot_episode1.robots.episode1_follower import (
    Episode1Follower,
    Episode1FollowerConfig,
)
from lerobot_robot_episode1.teleoperators.episode1_leader import (
    Episode1Leader,
    Episode1LeaderConfig,
)

E2E_FPS = 20
E2E_DURATION_S = 0.5  # 跑约 10 帧,足够验证链路且保持测试快


def test_official_teleop_loop_end_to_end(tmp_path, monkeypatch, fake_scservo):
    # --- 主臂:fake 舵机总线,端口用 /dev/null(存在即可,connect 只查存在性) ---
    leader = Episode1Leader(
        Episode1LeaderConfig(id="e2e_leader", port="/dev/null", calibration_dir=tmp_path)
    )
    leader.connect(calibrate=False)

    # --- 从臂:fake 上位机,回报位姿对齐主臂首帧动作(避开首帧防跳变阈值) ---
    first_action = leader.get_action()
    start_pose = [first_action[f"joint{i}.pos"] for i in range(1, 7)]
    fake = FakeEpisodeController(angles=start_pose)
    monkeypatch.setattr(follower_mod, "EpisodeClient", lambda **kwargs: fake)
    follower = Episode1Follower(
        Episode1FollowerConfig(id="e2e_follower", calibration_dir=tmp_path)
    )
    follower.connect()

    # --- 跑官方控制循环(官方管线 + 官方 teleop_loop,未改一行官方代码) ---
    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )
    teleop_loop(
        teleop=leader,
        robot=follower,
        fps=E2E_FPS,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
        display_data=False,
        duration=E2E_DURATION_S,
    )

    # --- 断言:从臂真的收到了动作,且关节键完整(下发格式:关节 '1'..'6' + 夹爪走 servo_gripper) ---
    assert len(fake.dynamic_moves) >= 3, f"循环帧数过少: {len(fake.dynamic_moves)}"
    assert {str(i) for i in range(1, 7)} <= set(fake.dynamic_moves[-1].keys())
    assert len(fake.sent_gripper) >= 1, "夹爪指令未下发"

    # --- 干净断开(disconnect 幂等,重复调不炸) ---
    leader.disconnect()
    follower.disconnect()
    follower.disconnect()
    assert fake.synced >= 1  # 保留了 fork 的 sync_motor_angles 收尾
