"""EpisodeClient（TCP）协议测试：编解码、超时、长度上限、重连。mock server，无真机。"""

import socket
import time

import pytest

from conftest import MockTCPServer
from lerobot_robot_episode1.robots.episode1_follower.episode_client import (
    EpisodeClient,
    EpisodeClientError,
    EpisodeProtocolError,
)


def test_roundtrip(tcp_server):
    tcp_server.responder = lambda cmd: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    client = EpisodeClient(ip="127.0.0.1", port=tcp_server.port)
    assert client.get_motor_angles() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert tcp_server.commands[-1] == {"action": "get_motor_angles"}
    client.close()


def test_persistent_connection_reused(tcp_server):
    client = EpisodeClient(ip="127.0.0.1", port=tcp_server.port)
    client.sync_motor_angles()
    client.servo_gripper(50)
    assert len(tcp_server.commands) == 2
    client.close()


def test_oversized_response_length_rejected():
    # 畸形包：声明 10MB 响应 → 协议错误，断开报错
    server = MockTCPServer(responder=lambda cmd: ("raw", (10_000_000).to_bytes(8, "big") + b"x" * 16)).start()
    try:
        client = EpisodeClient(ip="127.0.0.1", port=server.port)
        with pytest.raises(EpisodeProtocolError, match="超过上限"):
            client.get_motor_angles()
    finally:
        server.close()


def test_invalid_json_rejected():
    server = MockTCPServer(responder=lambda cmd: ("raw", (3).to_bytes(8, "big") + b"xyz")).start()
    try:
        client = EpisodeClient(ip="127.0.0.1", port=server.port)
        with pytest.raises(EpisodeProtocolError, match="JSON"):
            client.get_motor_angles()
    finally:
        server.close()


def test_oversized_outgoing_command_rejected(tcp_server):
    client = EpisodeClient(ip="127.0.0.1", port=tcp_server.port, max_message_bytes=64)
    with pytest.raises(EpisodeProtocolError, match="拒绝发送"):
        client.dynamic_move(goal_pos={"1": 1.0}, current_joint_pulses=[0] * 6, motors_max_speed=[1.0] * 6)
    client.close()


def test_timeout_raises_after_max_attempts():
    def slow(cmd):
        time.sleep(1.0)
        return {"ok": True}

    server = MockTCPServer(responder=slow).start()
    try:
        client = EpisodeClient(ip="127.0.0.1", port=server.port, timeout=0.1, max_attempts=2)
        start = time.perf_counter()
        with pytest.raises(EpisodeClientError):
            client.get_motor_angles()
        # 2 次尝试 × 0.1s 超时，远小于服务端 1s 延迟
        assert time.perf_counter() - start < 1.0
    finally:
        server.close()


def test_reconnect_after_server_drop():
    # 第一次连接被服务端断开 → 自动重连一次后成功
    state = {"calls": 0}

    def flaky(cmd):
        state["calls"] += 1
        if state["calls"] == 1:
            return None  # 直接断开，不响应
        return [0.0] * 6

    server = MockTCPServer(responder=flaky).start()
    try:
        client = EpisodeClient(ip="127.0.0.1", port=server.port, timeout=0.5, max_attempts=2)
        assert client.get_motor_angles() == [0.0] * 6
        assert state["calls"] == 2
        client.close()
    finally:
        server.close()


def test_reconnect_limit_exceeded():
    server = MockTCPServer(responder=lambda cmd: None).start()  # 永远断开
    try:
        client = EpisodeClient(ip="127.0.0.1", port=server.port, timeout=0.2, max_attempts=3)
        with pytest.raises(EpisodeClientError, match="3 次尝试"):
            client.get_motor_angles()
    finally:
        server.close()


def test_connect_refused_raises():
    # 找一个没监听的端口
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    client = EpisodeClient(ip="127.0.0.1", port=port, timeout=0.2, max_attempts=2)
    with pytest.raises(EpisodeClientError):
        client.get_motor_angles()
