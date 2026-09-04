"""episode1-pose-check 的纯函数测试：逐关节比较与判定。"""
from lerobot_robot_episode1.cli.pose_check import compare, render

STARTUP2 = [180, 90, 83, 210, 20, 210]


def _action(**over):
    a = {f"joint{i}.pos": v for i, v in zip(range(1, 7), STARTUP2)}
    a["gripper.pos"] = 40.0
    a.update({f"{k}.pos": v for k, v in over.items()})
    return a


def test_all_green_when_leader_matches_startup_pose():
    rows = compare(_action(), STARTUP2, 30.0)
    assert all(ok for *_, ok in rows)
    assert "全部 ✅" in render(rows, 40.0, None)


def test_flags_the_joint_that_is_off_like_the_real_runs():
    # 2026-09-04 真机：主臂完全竖直 → joint5 目标 101.9 vs 启动位姿 20
    rows = compare(_action(joint5=101.9), STARTUP2, 30.0)
    bad = [j for j, *_, ok in rows if not ok]
    assert bad == ["joint5"]
    # 第二次：转错成小臂 → joint3 目标 163（量程上限）
    rows = compare(_action(joint3=163.0), STARTUP2, 30.0)
    assert [j for j, *_, ok in rows if not ok] == ["joint3"]
    out = render(rows, 40.0, [180, 90, 83, 30, 110, 30])
    assert "⛔ 调这个" in out and "从臂此刻" in out
