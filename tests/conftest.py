"""共享测试工装：fake scservo_sdk、fake 主臂总线、fake 上位机、mock TCP server。全部无硬件。"""

import json
import socket
import sys
import threading
import types

import numpy as np
import pytest


# ---------------------------------------------------------------- fake scservo_sdk


class FakeGroupSyncRead:
    def __init__(self, port_handler, packet_handler, addr, size):
        self.addr = addr
        self.size = size
        self.ids = []

    def addParam(self, idx):
        self.ids.append(idx)

    def txRxPacket(self):
        return 0  # COMM_SUCCESS

    def getData(self, idx, addr, size):
        return FakeScservoModule.POSITIONS.get(idx, 0)


class FakeGroupSyncWrite:
    def __init__(self, port_handler, packet_handler, addr, size):
        self.addr = addr
        self.size = size
        self.params = {}

    def addParam(self, idx, data):
        self.params[idx] = data

    def changeParam(self, idx, data):
        self.params[idx] = data

    def txPacket(self):
        FakeScservoModule.WRITES.append(dict(self.params))
        return 0  # COMM_SUCCESS


class FakePortHandler:
    def __init__(self, port):
        self.port_name = port
        self.ser = types.SimpleNamespace(reset_output_buffer=lambda: None, reset_input_buffer=lambda: None)
        self._baudrate = 1_000_000
        self.is_open = False

    def openPort(self):
        self.is_open = True
        return True

    def closePort(self):
        self.is_open = False

    def setPacketTimeoutMillis(self, ms):
        pass

    def getBaudRate(self):
        return self._baudrate

    def setBaudRate(self, baudrate):
        self._baudrate = baudrate
        return True


class FakePacketHandler:
    def __init__(self, protocol_version):
        self.protocol_version = protocol_version

    def getTxRxResult(self, comm):
        return f"comm={comm}"


class FakeScservoModule(types.ModuleType):
    """按 motor_id 提供位置读数的 fake scservo_sdk。POSITIONS/WRITES 由测试设置。"""

    POSITIONS: dict[int, int] = {}
    WRITES: list[dict] = []

    def __init__(self):
        super().__init__("scservo_sdk")
        self.COMM_SUCCESS = 0
        self.PortHandler = FakePortHandler
        self.PacketHandler = FakePacketHandler
        self.GroupSyncRead = FakeGroupSyncRead
        self.GroupSyncWrite = FakeGroupSyncWrite
        self.SCS_LOWORD = lambda v: v & 0xFFFF
        self.SCS_HIWORD = lambda v: (v >> 16) & 0xFFFF
        self.SCS_LOBYTE = lambda v: v & 0xFF
        self.SCS_HIBYTE = lambda v: (v >> 8) & 0xFF


@pytest.fixture()
def fake_scservo(monkeypatch):
    module = FakeScservoModule()
    FakeScservoModule.POSITIONS = {i: 2048 for i in range(1, 8)}
    FakeScservoModule.WRITES = []
    monkeypatch.setitem(sys.modules, "scservo_sdk", module)
    return module


# ---------------------------------------------------------------- fake 主臂总线（FeetechController 层）


class FakeLeaderMotorsBus:
    """Episode1Leader 体检用的 fake motors_bus。"""

    def __init__(self, fail_ids=()):
        self.fail_ids = set(fail_ids)
        self.port_handler = None  # 触发 leader 跳过 baudrate 设置

    def read_with_motor_ids(self, motor_models, motor_ids, data_name, num_retry=2):
        if motor_ids[0] in self.fail_ids:
            raise ConnectionError(f"ID {motor_ids[0]} 无响应")
        return [motor_ids[0]]


class FakeLeaderBus:
    """替换 Episode1Leader.bus 的 fake（FeetechController 接口）。"""

    def __init__(self, positions=None, fail_ids=()):
        self.positions = dict(positions) if positions else {i: 2048 for i in range(1, 8)}
        self.motors_bus = FakeLeaderMotorsBus(fail_ids)
        self.is_connected = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def read_position(self, motor_id):
        return self.positions.get(motor_id)

    def batch_read_positions(self):
        return dict(self.positions)


# ---------------------------------------------------------------- fake 上位机（EpisodeClient 层）


class FakeEpisodeController:
    """替换 EpisodeClient 的 fake 上位机。"""

    def __init__(self, angles=None):
        self.angles = list(angles) if angles else [180, 90, 83, 210, 110, 210]
        self.sent_gripper: list[int] = []
        self.dynamic_moves: list[dict] = []
        self.angle_mode_calls: list[list] = []
        self.synced = 0
        self.closed = False

    def sync_motor_angles(self):
        self.synced += 1
        return 0

    def angle_mode(self, angles, speed_ratio=1):
        self.angle_mode_calls.append(list(angles))
        return 0  # 预计运动时间 0 秒，不 sleep

    def servo_gripper(self, angle):
        self.sent_gripper.append(angle)
        return 1

    def get_motor_angles(self):
        return list(self.angles)

    def dynamic_move(self, goal_pos, current_joint_pulses, motors_max_speed):
        self.dynamic_moves.append(dict(goal_pos))
        return 0.05

    def close(self):
        self.closed = True


# ---------------------------------------------------------------- mock TCP server


class MockTCPServer:
    """长度前缀 + JSON 协议的 mock 上位机。

    responder(cmd: dict) 的返回值决定响应：
      - dict/list/数字等 → JSON 序列化后按协议返回
      - ("raw", bytes)   → 原样发送（构造畸形包用）
      - None             → 直接关闭连接（测试重连）
    """

    def __init__(self, responder=None):
        self.responder = responder or (lambda cmd: {"ok": True})
        self.commands: list[dict] = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen()
        self.port = self._sock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self._thread.start()
        return self

    @staticmethod
    def _recv(conn, n):
        data = b""
        while len(data) < n:
            chunk = conn.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _serve(self):
        self._sock.settimeout(0.2)
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(5)
            with conn:
                while not self._stop.is_set():
                    try:
                        hdr = self._recv(conn, 8)
                        if hdr is None:
                            break
                        payload = self._recv(conn, int.from_bytes(hdr, "big"))
                        if payload is None:
                            break
                        cmd = json.loads(payload)
                        self.commands.append(cmd)
                        resp = self.responder(cmd)
                    except (ConnectionError, socket.timeout, json.JSONDecodeError):
                        break
                    if resp is None:
                        break  # 不响应，直接断开
                    if isinstance(resp, tuple) and resp[0] == "raw":
                        conn.sendall(resp[1])
                        continue
                    data = json.dumps(resp).encode()
                    conn.sendall(len(data).to_bytes(8, "big") + data)

    def close(self):
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=2)


@pytest.fixture()
def tcp_server():
    server = MockTCPServer().start()
    yield server
    server.close()
