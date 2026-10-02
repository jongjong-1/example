"""
TSA 그리퍼 역구동성(Backdrivability) 실험 GUI

실험 순서 (버튼 번호 순서대로)
  1. Close Gripper  : TSA 그리퍼를 힘 명령으로 오므림
  2. Zero Current   : TSA 모터 전류(힘 명령)를 0 으로 만듦
  3. Start Pull     : 다이나믹셀이 "전류 제어"로 당김  (2번을 해야만 활성화됨)
                      전류는 0 에서 Pull Current 까지 Ramp 시간 동안 서서히 올라감
  -  Stop Pull      : 당기기 정지 (다이나믹셀 전류 0)
  -  DXL Return     : 다이나믹셀을 당기기 전 위치로 복귀 후 토크 OFF
  -  Stop & Save    : 기록 정지 + CSV 저장(backdrive_data 폴더) + 결과 그래프

기록 데이터 (100 Hz)
  - 다이나믹셀 엔코더 (tick, deg), 다이나믹셀 전류 (명령값 / 측정값) [mA]
  - FT 센서 z축 힘 Fz [N]
  - TSA 모터 엔코더, TSA 모터 전류 [A]
  - (참고용) TSA Reference Force / Output Force

※ 프로그램을 실행해도 다이나믹셀은 토크 OFF 상태로 값만 읽고, 움직이지 않습니다.
※ 전류 제어는 스스로 멈추지 않으므로, Max Angle / Fz Limit 을 넘으면 자동으로 전류를 0 으로 만듭니다.
"""

import can
import struct
import subprocess
import time
import threading
import sys
import os
import csv
import math
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    DXL_SDK_OK = True
except ImportError:
    DXL_SDK_OK = False


# =====================================================================
#  설정 (CONFIG)  ←  본인 하드웨어에 맞게 여기만 고치면 됩니다
# =====================================================================

# ---- TSA 그리퍼 (CAN) ----
CAN_CHANNEL = 'can0'
CAN_BITRATE = 1000000

ID_TSA_CMD       = 0x100   # PC -> 그리퍼 : 힘 명령 (float32), b'return'
ID_TSA_REF_FORCE = 0x201   # 그리퍼 -> PC : Reference Force (float32)
ID_TSA_OUT_FORCE = 0x202   # 그리퍼 -> PC : Output Force (float32)
ID_TSA_ENCODER   = 0x203   # 그리퍼 -> PC : TSA 모터 엔코더 (float32)  ★가정: 펌웨어에서 이 ID로 보내야 함
ID_TSA_CURRENT   = 0x204   # 그리퍼 -> PC : TSA 모터 전류 [A] (float32) ★가정: 펌웨어에서 이 ID로 보내야 함

CLOSE_FORCE_DEFAULT = 5.0    # 그리퍼 오므릴 때 기본 힘 [N]
CLOSE_FORCE_MAX     = 14.0   # 힘 명령 상한 [N]
ZERO_SETTLE_TIME    = 1.0    # 전류 0 명령 후 Pull 버튼이 켜질 때까지 대기 [s]

# ---- 다이나믹셀: MX-64 (Protocol 2.0 펌웨어, model number 311) ----
DXL_PORT       = '/dev/ttyUSB0'
DXL_BAUD       = 3000000
DXL_ID         = 1
DXL_PROTOCOL   = 2.0
PULL_DIRECTION = +1          # 당기는 방향이 반대면 -1 로 변경

PULL_CURRENT_DEFAULT   = 200.0    # 당기는 전류 기본값 [mA]
PULL_CURRENT_MAX       = 2000.0   # 입력 가능한 전류 상한 [mA] (오타 방지용)
PULL_RAMP_DEFAULT      = 5.0      # 0 → Pull Current 까지 올리는 시간 [s] (0 이면 바로 인가)
PULL_ANGLE_MAX_DEFAULT = 180.0    # 이 각도 이상 돌아가면 자동 정지 [deg]
RETURN_SPEED_RPM       = 5.0      # DXL Return 속도 [rpm]

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_CURRENT     = 102     # 2 byte
ADDR_PROFILE_VELOCITY = 112     # 4 byte
ADDR_GOAL_POSITION    = 116     # 4 byte
ADDR_PRESENT_CURRENT  = 126     # 2 byte  (126~135: 전류 2 + 속도 4 + 위치 4 를 한 번에 읽음)
MODE_CURRENT          = 0       # 전류 제어 (당길 때)
MODE_EXT_POSITION     = 4       # 확장 위치제어 (복귀할 때)
TICKS_PER_REV         = 4096
CURRENT_UNIT_MA       = 3.36    # MX-64 : 전류 1단위 = 3.36 mA
RPM_PER_UNIT          = 0.229   # Profile Velocity 1단위 = 0.229 rpm
REACH_TOL_TICK        = 10      # 목표 도달 판정 오차 [tick]

# ---- FT 센서 (Robotous RFT 시리즈, CAN 출력 기준) ★가정 ----
FT_ENABLE        = True
FT_CMD_ID        = 0x64    # PC -> 센서 명령 ID
FT_DATA_ID       = 0x01    # 센서 -> PC 데이터 첫 번째 프레임 ID
FT_FORCE_DIVIDER = 50.0    # raw / 50 = [N]
FZ_LIMIT_DEFAULT = 20.0    # |Fz| 가 이 값을 넘으면 자동으로 당기기 정지 [N]

# ---- 기록 / 화면 ----
TIME_WINDOW = 10.0   # 실시간 그래프 가로 범위 [s]
RECORD_DT   = 0.01   # 기록 주기 100 Hz (그래프와 상관없이 별도 스레드에서 기록)
PLOT_DT     = 0.1    # 화면 갱신 주기 10 Hz
SAVE_DIR    = os.path.join(os.path.dirname(os.path.abspath(__file__)), "backdrive_data")


# =====================================================================
#  실험 단계 (phase)  - CSV 에도 같이 저장됨
# =====================================================================
PHASE_IDLE, PHASE_CLOSE, PHASE_ZERO, PHASE_PULL, PHASE_HOLD, PHASE_RETURN = range(6)
PHASE_NAME = {
    PHASE_IDLE:   "IDLE",
    PHASE_CLOSE:  "CLOSE (gripper closing)",
    PHASE_ZERO:   "ZERO CURRENT",
    PHASE_PULL:   "PULLING (current ctrl)",
    PHASE_HOLD:   "PULL STOPPED",
    PHASE_RETURN: "DXL RETURNING",
}

# ================= 공유 변수 =================
NAN = float('nan')

last_ref_force = NAN
last_out_force = NAN
last_tsa_enc   = NAN
last_tsa_cur   = NAN
last_fz_raw    = NAN     # 영점 보정 전 Fz
ft_offset      = 0.0     # FT Zero 버튼으로 잡은 영점
last_dxl_tick  = NAN     # 다이나믹셀 엔코더 [tick]
last_dxl_cur   = NAN     # 다이나믹셀 측정 전류 [mA]
dxl_goal_cur   = 0.0     # 다이나믹셀 명령 전류 [mA]

phase      = PHASE_IDLE
zero_ready = False       # True 일 때만 Start Pull 가능

# Start Pull 을 누를 때 입력칸에서 읽어 두는 값
pull_current   = PULL_CURRENT_DEFAULT
pull_ramp      = PULL_RAMP_DEFAULT
pull_angle_max = PULL_ANGLE_MAX_DEFAULT
fz_limit       = FZ_LIMIT_DEFAULT
pull_t_start    = 0.0
pull_start_tick = None
dxl_goal_tick   = None

pending_status = None    # 스레드 -> 화면으로 보내는 상태 메시지 (text, color)

t0 = time.time()
running   = True
recording = True

# ================= 데이터 버퍼 =================
buf_t, buf_phase = [], []
buf_dxl_tick, buf_dxl_deg = [], []
buf_dxl_goal_cur, buf_dxl_cur = [], []
buf_fz = []
buf_tsa_enc, buf_tsa_cur = [], []
buf_ref_force, buf_out_force = [], []
buf_lock = threading.Lock()   # 기록 스레드와 그래프가 동시에 버퍼를 건드리지 않도록


# =====================================================================
#  CAN (TSA 그리퍼 + FT 센서)
# =====================================================================
def cmd(command):
    subprocess.run(command, shell=True, check=True)


def can_is_up():
    """can0 인터페이스가 이미 켜져 있는지 확인 (IFF_UP 플래그)"""
    try:
        with open(f"/sys/class/net/{CAN_CHANNEL}/flags") as f:
            return bool(int(f.read().strip(), 16) & 0x1)
    except (OSError, ValueError):
        return False


def can_setup():
    """can0 를 켜고 Bus 를 연다. 실패해도 프로그램은 계속 (다이나믹셀만이라도 쓰도록)"""
    if not os.path.exists(f"/sys/class/net/{CAN_CHANNEL}"):
        print(f"[CAN] {CAN_CHANNEL} 가 없습니다. PCAN-USB 연결 / 드라이버(peak_usb) 확인")
        return None

    if can_is_up():
        # 이미 켜져 있으면 sudo 없이 그대로 사용
        print(f"[CAN] 이미 켜져 있는 {CAN_CHANNEL} 를 사용합니다.")
    else:
        try:
            cmd(f"sudo ip link set {CAN_CHANNEL} down")
            # restart-ms 100 : BUS-OFF 가 되어도 0.1 s 뒤 자동 복구 (없으면 영원히 멈춤)
            cmd(f"sudo ip link set {CAN_CHANNEL} type can bitrate {CAN_BITRATE} restart-ms 100")
            cmd(f"sudo ip link set {CAN_CHANNEL} up")
            print(f"[CAN] {CAN_CHANNEL} 켬 ({CAN_BITRATE} bps)")
        except subprocess.CalledProcessError:
            print(f"""
[CAN] {CAN_CHANNEL} 를 켤 수 없습니다 (sudo 실패). TSA / FT 없이 계속합니다.
  일반 터미널에서 아래 명령으로 CAN 을 먼저 켠 뒤 다시 실행하세요.
      sudo ip link set {CAN_CHANNEL} up type can bitrate {CAN_BITRATE} restart-ms 100
""")
            return None
        time.sleep(0.1)

    try:
        return can.interface.Bus(channel=CAN_CHANNEL, interface='socketcan')
    except Exception as e:
        print(f"[CAN] Bus 열기 실패: {e}")
        return None


bus = can_setup()

can_rx_counts = {}         # 수신된 CAN ID 별 프레임 수 (연결 진단용)
can_tx_errors = 0          # 전송 실패 횟수
last_ft_time  = 0.0        # 마지막 FT 프레임 수신 시각
last_tsa_time = 0.0        # 마지막 TSA 프레임 수신 시각
can_state_text = "?"       # ip 명령으로 읽은 can state (ERROR-ACTIVE / BUS-OFF ...)


def can_send(arb_id, data):
    """CAN 전송. 버스가 없거나 전송 실패(ACK 없음, 버퍼 가득 등)해도 예외로 죽지 않음"""
    global can_tx_errors
    if bus is None:
        return False
    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False),
                 timeout=0.05)
        return True
    except Exception as e:
        can_tx_errors += 1
        if can_tx_errors <= 5 or can_tx_errors % 100 == 0:
            print(f"[CAN] 전송 실패 (ID 0x{arb_id:03X}, 누적 {can_tx_errors}회): {e}")
        return False


def send_force_value(f):
    """TSA 그리퍼에 힘 명령 전송 (0 을 보내면 전류 0)"""
    return can_send(ID_TSA_CMD, struct.pack('<f', f))


def ft_send_command(command_id):
    """Robotous RFT 명령 (0x0B: 연속 출력 시작, 0x0C: 정지)"""
    return can_send(FT_CMD_ID, [command_id, 0, 0, 0, 0, 0, 0, 0])


def decode_ft_fz(data):
    """
    FT 센서 CAN 프레임에서 Fz [N] 를 꺼내는 함수.
    ★ FT 센서가 다른 모델이면 이 함수와 위의 FT_* 설정만 바꾸면 됩니다.
    Robotous RFT: [0x0B, FxH, FxL, FyH, FyL, FzH, FzL, TxH]  (big-endian int16)
    """
    if len(data) < 7 or data[0] != 0x0B:
        return None
    raw = struct.unpack('>h', bytes(data[5:7]))[0]
    return raw / FT_FORCE_DIVIDER


def can_rx_thread():
    global last_ref_force, last_out_force, last_tsa_enc, last_tsa_cur, last_fz_raw
    global last_ft_time, last_tsa_time

    next_ft_retry = time.time() + 1.0
    while running:
        # FT 데이터가 1초 넘게 안 오면 연속 출력 시작 명령을 다시 보냄
        # (센서 전원을 프로그램보다 늦게 켠 경우 등)
        now = time.time()
        if FT_ENABLE and now - last_ft_time > 1.0 and now > next_ft_retry:
            ft_send_command(0x0B)
            next_ft_retry = now + 1.0

        try:
            rx = bus.recv(timeout=0.1)
        except Exception as e:
            # 예전 코드는 여기서 break 해서 수신 스레드가 조용히 죽었음
            print(f"[CAN] 수신 오류: {e}")
            time.sleep(0.5)
            continue
        if rx is None or rx.is_error_frame or rx.is_remote_frame:
            continue

        aid = rx.arbitration_id
        can_rx_counts[aid] = can_rx_counts.get(aid, 0) + 1

        # ---- FT 센서 ----
        if FT_ENABLE and aid == FT_DATA_ID:
            fz = decode_ft_fz(rx.data)
            if fz is not None:
                last_fz_raw = fz
                last_ft_time = time.time()
            continue

        # ---- TSA 그리퍼 (모두 float32) ----
        if len(rx.data) < 4:
            continue
        value = struct.unpack('<f', bytes(rx.data[:4]))[0]

        if aid == ID_TSA_REF_FORCE:
            last_ref_force = value
        elif aid == ID_TSA_OUT_FORCE:
            last_out_force = value
        elif aid == ID_TSA_ENCODER:
            last_tsa_enc = value
        elif aid == ID_TSA_CURRENT:
            last_tsa_cur = value
        else:
            continue
        last_tsa_time = time.time()


def can_state_thread():
    """1초마다 can0 상태(ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF)를 읽어 화면에 표시"""
    global can_state_text
    while running:
        try:
            out = subprocess.run(["ip", "-details", "link", "show", CAN_CHANNEL],
                                 capture_output=True, text=True, timeout=1).stdout
            state = "DOWN" if "state DOWN" in out else "?"
            for line in out.splitlines():
                words = line.split()
                if words[:2] == ["can", "state"] and len(words) > 2:
                    state = words[2]
                    # 예: "can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100"
                    for i, w in enumerate(words):
                        if w.endswith("berr-counter") and len(words) > i + 4:
                            state += f" (tx err {words[i + 2]}, rx err {words[i + 4].rstrip(')')})"
            can_state_text = state
        except Exception:
            pass
        time.sleep(1.0)


# =====================================================================
#  다이나믹셀
# =====================================================================
dxl_port = None
dxl_packet = None
dxl_ok = False

# 다이나믹셀 통신과 phase 변경을 한 묶음으로 보호하는 잠금.
# (Stop 을 누른 직후에 스레드가 다시 전류 명령을 보내는 일이 없도록)
dxl_lock = threading.RLock()


def tick_to_deg(tick):
    return tick * 360.0 / TICKS_PER_REV


def dxl_write(addr, size, value):
    """size: 1 / 2 / 4 바이트. 성공하면 True"""
    if not dxl_ok:
        return False
    with dxl_lock:
        if size == 1:
            res, err = dxl_packet.write1ByteTxRx(dxl_port, DXL_ID, addr, value)
        elif size == 2:
            res, err = dxl_packet.write2ByteTxRx(dxl_port, DXL_ID, addr, value & 0xFFFF)
        else:
            res, err = dxl_packet.write4ByteTxRx(dxl_port, DXL_ID, addr, value & 0xFFFFFFFF)
    if res != COMM_SUCCESS:
        print(f"[DXL] 통신 실패 (addr {addr}): {dxl_packet.getTxRxResult(res)}")
        return False
    if err != 0:
        print(f"[DXL] 에러 응답 (addr {addr}): {dxl_packet.getRxPacketError(err)}")
        return False
    return True


def dxl_read_state():
    """(전류[mA], 위치[tick]) 를 한 번에 읽음. 실패하면 None"""
    with dxl_lock:
        data, res, err = dxl_packet.readTxRx(dxl_port, DXL_ID, ADDR_PRESENT_CURRENT, 10)
    if res != COMM_SUCCESS or len(data) < 10:
        return None
    cur = int.from_bytes(bytes(data[0:2]), 'little', signed=True) * CURRENT_UNIT_MA
    pos = int.from_bytes(bytes(data[6:10]), 'little', signed=True)
    return cur, pos


def dxl_torque(on):
    return dxl_write(ADDR_TORQUE_ENABLE, 1, 1 if on else 0)


_last_cur_unit = None   # 마지막으로 보낸 전류 명령 (같은 값을 반복해서 보내지 않기 위함)


def dxl_set_mode(mode):
    """제어 모드 변경 (토크 OFF 상태에서만 바꿀 수 있음)"""
    global _last_cur_unit
    _last_cur_unit = None   # 모드를 바꾼 뒤에는 전류 명령을 반드시 다시 보내도록
    return dxl_torque(False) and dxl_write(ADDR_OPERATING_MODE, 1, mode)


def dxl_set_current(ma):
    """전류 명령 [mA]. 값이 바뀔 때만 실제로 전송"""
    global dxl_goal_cur, _last_cur_unit
    unit = int(round(ma / CURRENT_UNIT_MA))
    if unit == _last_cur_unit:
        return True
    if not dxl_write(ADDR_GOAL_CURRENT, 2, unit):
        return False
    _last_cur_unit = unit
    dxl_goal_cur = unit * CURRENT_UNIT_MA
    return True


def dxl_set_speed(rpm):
    unit = max(1, int(round(rpm / RPM_PER_UNIT)))   # 0 은 '최대 속도' 라서 최소 1
    return dxl_write(ADDR_PROFILE_VELOCITY, 4, unit)


def dxl_goto(tick):
    return dxl_write(ADDR_GOAL_POSITION, 4, int(tick))


def set_usb_low_latency():
    """USB-시리얼 지연(기본 16 ms)을 1 ms 로 줄여서 다이나믹셀을 빠르게 읽을 수 있게 함"""
    path = f"/sys/bus/usb-serial/devices/{os.path.basename(DXL_PORT)}/latency_timer"
    try:
        with open(path) as f:
            if f.read().strip() == "1":
                return
        cmd(f"echo 1 | sudo tee {path} > /dev/null")
    except Exception:
        print("[DXL] latency_timer 를 1 ms 로 못 바꿈 → 다이나믹셀 값이 약 30~60 Hz 로만 갱신됩니다.")


DXL_SCAN_BAUDS = [3000000, 57600, 1000000, 2000000, 4000000, 115200, 4500000]
ARDUINO_VID    = 0x2341     # Arduino 포트는 다이나믹셀 검색에서 제외 (열면 보드가 리셋될 수 있음)


def dxl_candidate_ports():
    """설정된 DXL_PORT 를 먼저, 그 다음 다른 USB 시리얼 포트 (Arduino 제외)"""
    ports = [DXL_PORT]
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.vid == ARDUINO_VID or "arduino" in f"{p.description} {p.manufacturer}".lower():
                continue
            if ("ttyUSB" in p.device or "ttyACM" in p.device) and p.device not in ports:
                ports.append(p.device)
    except ImportError:
        pass
    return ports


def dxl_try(port, baud):
    """port/baud 에서 다이나믹셀을 찾음. 찾으면 (PortHandler, ID, model), 못 찾으면 None"""
    ph = PortHandler(port)
    try:
        if not ph.openPort():
            return None
    except Exception as e:
        print(f"[DXL] {port} 열기 실패: {e}")
        return None
    if not ph.setBaudRate(baud):
        ph.closePort()
        return None
    # 1) 설정된 ID 로 ping
    model, res, _ = dxl_packet.ping(ph, DXL_ID)
    if res == COMM_SUCCESS:
        return ph, DXL_ID, model
    # 2) broadcast ping 으로 아무 ID 나 (Protocol 2.0)
    found, res = dxl_packet.broadcastPing(ph)
    if res == COMM_SUCCESS and found:
        dxl_id = sorted(found)[0]
        return ph, dxl_id, found[dxl_id][0]
    ph.closePort()
    return None


def dxl_init():
    """포트만 열고 토크 OFF. 모터는 움직이지 않음.
    설정값(DXL_PORT / DXL_BAUD / DXL_ID)으로 안 되면 다른 포트/baud/ID 를 자동으로 찾음."""
    global dxl_port, dxl_packet, dxl_ok, DXL_PORT, DXL_BAUD, DXL_ID

    if not DXL_SDK_OK:
        print("[DXL] dynamixel_sdk 가 없습니다:  pip install dynamixel-sdk")
        return

    dxl_packet = PacketHandler(DXL_PROTOCOL)
    if not os.path.exists(DXL_PORT):
        print(f"[DXL] {DXL_PORT} 가 없습니다. U2D2 USB 연결 확인. 다른 포트를 찾아봅니다.")
    elif not os.access(DXL_PORT, os.R_OK | os.W_OK):
        print(f"[DXL] {DXL_PORT} 권한 없음:  sudo usermod -aG dialout $USER  후 재로그인")

    result = None
    try:
        # 먼저 설정값 그대로 시도
        if os.path.exists(DXL_PORT):
            result = dxl_try(DXL_PORT, DXL_BAUD)
            if result:
                port, baud = DXL_PORT, DXL_BAUD
        # 안 되면 자동 검색
        if not result:
            print(f"[DXL] {DXL_PORT} / {DXL_BAUD} bps / ID {DXL_ID} 응답 없음 → 자동 검색 중...")
            for port in dxl_candidate_ports():
                if not os.path.exists(port):
                    continue
                for baud in DXL_SCAN_BAUDS:
                    result = dxl_try(port, baud)
                    if result:
                        break
                if result:
                    break
    except Exception as e:
        print(f"[DXL] 초기화 실패: {e}")
        result = None

    if not result:
        print("[DXL] 다이나믹셀을 찾지 못했습니다. 확인할 것:\n"
              "      - 다이나믹셀 전원(12V) 이 켜져 있는지 (U2D2 USB 만으로는 전원이 안 들어감)\n"
              "      - Dynamixel Wizard 등 다른 프로그램이 포트를 쓰고 있지 않은지\n"
              "      - python3 check_connection.py 로 검색되는지")
        return

    dxl_port, found_id, model = result
    if (port, baud, found_id) != (DXL_PORT, DXL_BAUD, DXL_ID):
        print(f"[DXL] 설정과 다른 곳에서 찾음 → CONFIG 를 고쳐 두세요: "
              f"DXL_PORT='{port}', DXL_BAUD={baud}, DXL_ID={found_id}")
    DXL_PORT, DXL_BAUD, DXL_ID = port, baud, found_id

    set_usb_low_latency()
    res, err = dxl_packet.write1ByteTxRx(dxl_port, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    if res != COMM_SUCCESS:
        print(f"[DXL] 토크 OFF 명령 실패: {dxl_packet.getTxRxResult(res)}")
        dxl_port.closePort()
        return
    if err != 0:
        # 하드웨어 에러(과부하 등) 상태면 재부팅해야 토크를 켤 수 있음
        print(f"[DXL] 하드웨어 에러 상태: {dxl_packet.getRxPacketError(err)} → reboot 시도")
        dxl_packet.reboot(dxl_port, DXL_ID)
        time.sleep(1.0)
    dxl_ok = True
    print(f"[DXL] 연결됨: {DXL_PORT}, {DXL_BAUD} bps, ID {DXL_ID}, model {model} (토크 OFF)")


def dxl_thread():
    """다이나믹셀 값 읽기 + 당기는 중 전류 명령/안전 확인 + 복귀 완료 확인"""
    global last_dxl_tick, last_dxl_cur, phase

    while running:
        with dxl_lock:
            state = dxl_read_state()
            if state is not None:
                last_dxl_cur, last_dxl_tick = state

            # ---- 당기는 중 ----
            if phase == PHASE_PULL:
                fz = last_fz_raw - ft_offset
                moved = abs(tick_to_deg(last_dxl_tick - pull_start_tick))
                if moved > pull_angle_max:
                    stop_pull(f"Max Angle ({pull_angle_max:.0f} deg) 도달로 자동 정지")
                elif not math.isnan(fz) and abs(fz) > fz_limit:
                    stop_pull(f"Fz limit ({fz_limit:.1f} N) 초과로 자동 정지")
                else:
                    # 전류를 0 → pull_current 까지 서서히 올림
                    elapsed = time.time() - pull_t_start
                    ratio = 1.0 if pull_ramp <= 0 else min(1.0, elapsed / pull_ramp)
                    dxl_set_current(PULL_DIRECTION * pull_current * ratio)

            # ---- 복귀 중: 도착하면 토크 OFF ----
            elif phase == PHASE_RETURN:
                if abs(last_dxl_tick - dxl_goal_tick) <= REACH_TOL_TICK:
                    dxl_torque(False)
                    phase = PHASE_IDLE
                    set_status("DXL 복귀 완료 (토크 OFF). 다시 하려면 [1. Close Gripper] 부터")

        time.sleep(0.002)


# =====================================================================
#  버튼 동작
# =====================================================================
def set_status(text, color="#00ff88"):
    """상태 메시지 예약. 실제 화면 반영은 update_gui() 에서 함 (스레드에서 불러도 안전)"""
    global pending_status
    pending_status = (text, color)


def refresh_buttons():
    """현재 phase 에 따라 누를 수 있는 버튼만 활성화"""
    def enable(btn, on):
        btn.state(["!disabled"] if on else ["disabled"])

    dxl_busy = phase in (PHASE_PULL, PHASE_HOLD, PHASE_RETURN)
    enable(btn_close, not dxl_busy)
    enable(btn_tsa_return, not dxl_busy)
    enable(btn_pull, dxl_ok and zero_ready and phase == PHASE_ZERO)
    enable(btn_stop_pull, phase == PHASE_PULL)
    enable(btn_dxl_return, phase == PHASE_HOLD)


def close_gripper():
    """1단계: 그리퍼 오므리기"""
    global phase, zero_ready
    try:
        f = float(close_force_entry.get())
    except ValueError:
        set_status("Close Force 값이 잘못됨", "red")
        return
    f = min(max(f, 0.0), CLOSE_FORCE_MAX)

    if not send_force_value(f):
        set_status("CAN 전송 실패 - TSA 에 명령이 안 감 (CAN 상태 확인)", "red")
        return
    phase = PHASE_CLOSE
    zero_ready = False
    set_status(f"1) Closing gripper: {f:.2f} N  →  다 오므려지면 [2. Zero Current]")
    refresh_buttons()


def zero_current():
    """2단계: TSA 전류 0"""
    global phase, zero_ready
    if not send_force_value(0.0):
        set_status("CAN 전송 실패 - TSA 전류 0 명령이 안 감 (CAN 상태 확인)", "red")
        return

    if phase in (PHASE_PULL, PHASE_HOLD, PHASE_RETURN):
        return   # 당기는 도중에 눌렀으면 0 명령만 다시 보냄

    phase = PHASE_ZERO
    zero_ready = False
    set_status("2) TSA current = 0 명령 전송, 안정화 대기 중...")
    refresh_buttons()
    root.after(int(ZERO_SETTLE_TIME * 1000), zero_settled)


def zero_settled():
    global zero_ready
    if phase != PHASE_ZERO:
        return
    zero_ready = True
    if dxl_ok:
        set_status("2) TSA current = 0  →  [3. Start Pull] 가능")
    else:
        set_status("TSA current = 0 (다이나믹셀 연결 안 됨)", "orange")
    refresh_buttons()


def start_pull():
    """3단계: 다이나믹셀 전류 제어로 당기기"""
    global phase, zero_ready, pull_start_tick, pull_t_start
    global pull_current, pull_ramp, pull_angle_max, fz_limit

    if not (dxl_ok and zero_ready and phase == PHASE_ZERO):
        return
    try:
        current = float(pull_current_entry.get())
        ramp = float(pull_ramp_entry.get())
        angle_max = float(pull_angle_entry.get())
        limit = float(fz_limit_entry.get())
    except ValueError:
        set_status("Pull 설정값이 잘못됨", "red")
        return
    if not (0 < current <= PULL_CURRENT_MAX):
        set_status(f"Pull Current 는 0 ~ {PULL_CURRENT_MAX:.0f} mA 사이여야 함", "red")
        return
    if ramp < 0 or angle_max <= 0:
        set_status("Ramp 는 0 이상, Max Angle 은 0 보다 커야 함", "red")
        return

    send_force_value(0.0)   # 혹시 모르니 TSA 전류 0 한 번 더

    with dxl_lock:
        # 전류 제어 모드로 바꾸고, 전류 0 인 상태로 토크 ON
        ok = dxl_set_mode(MODE_CURRENT) and dxl_set_current(0.0) and dxl_torque(True)
        state = dxl_read_state() if ok else None
        if state is None:
            dxl_torque(False)
            set_status("다이나믹셀 통신 오류 (터미널 메시지 확인)", "red")
            return

        pull_current, pull_ramp, pull_angle_max, fz_limit = current, ramp, angle_max, limit
        pull_start_tick = state[1]
        pull_t_start = time.time()
        phase = PHASE_PULL
        zero_ready = False

    set_status(f"3) Pulling: 0 → {current:.0f} mA ({ramp:.1f} s ramp)")
    refresh_buttons()


def stop_pull(reason="Stop Pull"):
    """당기기 정지: 다이나믹셀 전류 0"""
    global phase
    with dxl_lock:
        if phase != PHASE_PULL:
            return
        if not dxl_set_current(0.0):
            dxl_torque(False)       # 전류 0 명령이 실패하면 토크를 꺼 버림
        phase = PHASE_HOLD
    set_status(f"{reason} (DXL 전류 0)  →  [DXL Return] 으로 원위치", "orange")


def dxl_return():
    """다이나믹셀을 당기기 전 위치로 복귀 (위치제어 모드로 바꿔서 이동)"""
    global phase, dxl_goal_tick

    with dxl_lock:
        if phase != PHASE_HOLD or pull_start_tick is None:
            return

        before = dxl_read_state()
        ok = dxl_set_mode(MODE_EXT_POSITION)
        after = dxl_read_state()
        if not ok or before is None or after is None:
            dxl_torque(False)
            set_status("다이나믹셀 통신 오류 (터미널 메시지 확인)", "red")
            return

        # 모드를 바꿀 때 위치값이 한 바퀴 이내로 초기화되는 경우에도 같은 양만큼 되돌아가게 함
        if abs(after[1] - before[1]) < TICKS_PER_REV // 2:
            goal = pull_start_tick
        else:
            goal = after[1] - (before[1] - pull_start_tick)

        if not (dxl_set_speed(RETURN_SPEED_RPM) and dxl_torque(True) and dxl_goto(goal)):
            dxl_torque(False)
            set_status("다이나믹셀 통신 오류 (터미널 메시지 확인)", "red")
            return

        dxl_goal_tick = goal
        phase = PHASE_RETURN

    set_status("DXL returning...")
    refresh_buttons()


def tsa_return():
    """그리퍼 원위치 (기존 코드의 Return 과 동일한 CAN 메시지)"""
    global phase, zero_ready
    if not can_send(ID_TSA_CMD, b'return'):
        set_status("CAN 전송 실패 - TSA Return 명령이 안 감 (CAN 상태 확인)", "red")
        return
    phase = PHASE_IDLE
    zero_ready = False
    set_status("TSA RETURN Pos")
    refresh_buttons()


def ft_zero():
    """현재 Fz 를 0 으로 잡음 (소프트웨어 영점)"""
    global ft_offset
    if math.isnan(last_fz_raw):
        set_status("FT 센서 데이터 수신 없음", "red")
        return
    ft_offset = last_fz_raw
    set_status("FT Fz 영점 설정 완료")


def emergency_stop():
    global phase, zero_ready, _last_cur_unit, dxl_goal_cur
    with dxl_lock:
        phase = PHASE_IDLE
        zero_ready = False
        dxl_torque(False)        # 다이나믹셀 힘 풀기
        _last_cur_unit = None
        dxl_goal_cur = 0.0
    send_force_value(0.0)        # TSA 전류 0
    set_status("EMERGENCY STOP! DXL torque OFF, TSA force = 0", "red")
    refresh_buttons()


# =====================================================================
#  100 Hz 기록 스레드 (그래프 그리는 시간에 영향받지 않도록 화면과 분리)
# =====================================================================
def logger_thread():
    next_t = time.time()
    while running:
        with buf_lock:
            if recording:
                buf_t.append(time.time() - t0)
                buf_phase.append(phase)
                buf_dxl_tick.append(last_dxl_tick)
                buf_dxl_deg.append(tick_to_deg(last_dxl_tick))
                buf_dxl_goal_cur.append(dxl_goal_cur if phase == PHASE_PULL else 0.0)
                buf_dxl_cur.append(last_dxl_cur)
                buf_fz.append(last_fz_raw - ft_offset)
                buf_tsa_enc.append(last_tsa_enc)
                buf_tsa_cur.append(last_tsa_cur)
                buf_ref_force.append(last_ref_force)
                buf_out_force.append(last_out_force)

        next_t += RECORD_DT
        delay = next_t - time.time()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.time()


# =====================================================================
#  저장 / 결과 그래프
# =====================================================================
def save_csv():
    os.makedirs(SAVE_DIR, exist_ok=True)
    path = os.path.join(SAVE_DIR, time.strftime("backdrive_%Y%m%d_%H%M%S.csv"))
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "phase", "dxl_tick", "dxl_deg",
                    "dxl_goal_current_mA", "dxl_current_mA", "ft_fz_N",
                    "tsa_encoder", "tsa_current_A", "tsa_ref_force_N", "tsa_out_force_N"])
        for row in zip(buf_t, buf_phase, buf_dxl_tick, buf_dxl_deg,
                       buf_dxl_goal_cur, buf_dxl_cur, buf_fz,
                       buf_tsa_enc, buf_tsa_cur, buf_ref_force, buf_out_force):
            w.writerow(row)
    return path


def shade_pull(ax):
    """그래프에서 당기는 구간(PULL)을 노란색으로 표시"""
    start = None
    for t, p in zip(buf_t, buf_phase):
        if p == PHASE_PULL and start is None:
            start = t
        elif p != PHASE_PULL and start is not None:
            ax.axvspan(start, t, color="yellow", alpha=0.2)
            start = None
    if start is not None:
        ax.axvspan(start, buf_t[-1], color="yellow", alpha=0.2)


def stop_and_save():
    global recording

    if not recording:
        return

    # 안전하게 정지
    stop_pull("Stop & Save")
    send_force_value(0.0)
    with buf_lock:
        recording = False

    if len(buf_t) < 2:
        return

    path = save_csv()
    status_label.config(text=f"Saved: {os.path.basename(path)}", foreground="#00ff88")

    # ---- 그림 1: 시간 그래프 ----
    fig1, axes = plt.subplots(5, 1, figsize=(9, 10), sharex=True)
    fig1.suptitle("TSA Backdrivability Test (yellow = pulling)", fontsize=14)

    axes[0].plot(buf_t, buf_dxl_goal_cur, 'r--', linewidth=1.5, label="Goal")
    axes[0].plot(buf_t, buf_dxl_cur, 'm', linewidth=1.0, label="Present")
    axes[0].set_ylabel("DXL Current [mA]")
    axes[0].legend(loc="upper left")

    series = [
        (buf_dxl_deg, 'k', "DXL Encoder [deg]"),
        (buf_fz,      'b', "FT Fz [N]"),
        (buf_tsa_enc, 'c', "TSA Motor Encoder"),
        (buf_tsa_cur, 'g', "TSA Motor Current [A]"),
    ]
    for ax, (data, color, label) in zip(axes[1:], series):
        ax.plot(buf_t, data, color, linewidth=1.5)
        ax.set_ylabel(label)
    for ax in axes:
        ax.grid(True)
        shade_pull(ax)
    axes[-1].set_xlabel("Time [s]")
    fig1.tight_layout()

    # ---- 그림 2: 다이나믹셀이 당긴 각도(엔코더)에 대한 그래프 (PULL 구간만) ----
    idx = [i for i, p in enumerate(buf_phase) if p == PHASE_PULL]
    if len(idx) >= 2:
        disp = [(buf_dxl_deg[i] - buf_dxl_deg[idx[0]]) * PULL_DIRECTION for i in idx]
        fig2, bx = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
        fig2.suptitle("Pull phase only", fontsize=14)
        pull_series = [
            ([abs(buf_dxl_cur[i]) for i in idx], 'm.', "|DXL Current| [mA]"),
            ([buf_fz[i] for i in idx],           'b.', "FT Fz [N]"),
            ([buf_tsa_enc[i] for i in idx],      'c.', "TSA Motor Encoder"),
        ]
        for ax, (data, fmt_str, label) in zip(bx, pull_series):
            ax.plot(disp, data, fmt_str, markersize=3)
            ax.set_ylabel(label)
            ax.grid(True)
        bx[-1].set_xlabel("DXL Pull Displacement (encoder) [deg]")
        fig2.tight_layout()

    def on_fig_close(event):
        on_close()

    fig1.canvas.mpl_connect('close_event', on_fig_close)
    plt.show()


# =====================================================================
#  GUI
# =====================================================================
root = tk.Tk()
root.title("TSA Gripper Backdrivability Test")
root.configure(bg="black")
root.geometry("1250x900")
root.resizable(False, False)

style = ttk.Style()
style.theme_use("default")
style.configure("TFrame", background="black")
style.configure("TLabel", background="black", foreground="white")
style.configure("TButton", background="black", foreground="white")
style.map("TButton", foreground=[("disabled", "#555555")])
style.configure("TEntry", fieldbackground="black", foreground="white")
style.configure("Stop.TButton", foreground="#4da6ff")
style.configure("ESTOP.TButton", foreground="red", font=("TkDefaultFont", 10, "bold"))


def add_entry(parent, label, default, width=7):
    ttk.Label(parent, text=label).pack(side=tk.LEFT, padx=(8, 2))
    entry = ttk.Entry(parent, width=width)
    entry.pack(side=tk.LEFT)
    entry.insert(0, str(default))
    return entry


# ---- 1줄: TSA 그리퍼 ----
row1 = ttk.Frame(root)
row1.pack(fill=tk.X, padx=10, pady=4)
ttk.Label(row1, text="[TSA]", font=("TkDefaultFont", 10, "bold")).pack(side=tk.LEFT)
close_force_entry = add_entry(row1, "Close Force (N):", CLOSE_FORCE_DEFAULT)

btn_close = ttk.Button(row1, text="1. Close Gripper", command=close_gripper)
btn_close.pack(side=tk.LEFT, padx=6)
btn_zero = ttk.Button(row1, text="2. Zero Current", command=zero_current)
btn_zero.pack(side=tk.LEFT, padx=6)
btn_tsa_return = ttk.Button(row1, text="TSA Return", command=tsa_return)
btn_tsa_return.pack(side=tk.LEFT, padx=6)
ttk.Button(row1, text="E-STOP", command=emergency_stop,
           style="ESTOP.TButton").pack(side=tk.LEFT, padx=12)

# ---- 2줄: 다이나믹셀 ----
row2 = ttk.Frame(root)
row2.pack(fill=tk.X, padx=10, pady=4)
ttk.Label(row2, text="[DXL]", font=("TkDefaultFont", 10, "bold")).pack(side=tk.LEFT)
pull_current_entry = add_entry(row2, "Pull Current (mA):", PULL_CURRENT_DEFAULT)
pull_ramp_entry    = add_entry(row2, "Ramp (s):", PULL_RAMP_DEFAULT, width=5)
pull_angle_entry   = add_entry(row2, "Max Angle (deg):", PULL_ANGLE_MAX_DEFAULT)
fz_limit_entry     = add_entry(row2, "Fz Limit (N):", FZ_LIMIT_DEFAULT, width=5)

btn_pull = ttk.Button(row2, text="3. Start Pull", command=start_pull)
btn_pull.pack(side=tk.LEFT, padx=6)
btn_stop_pull = ttk.Button(row2, text="Stop Pull", command=stop_pull)
btn_stop_pull.pack(side=tk.LEFT, padx=6)
btn_dxl_return = ttk.Button(row2, text="DXL Return", command=dxl_return)
btn_dxl_return.pack(side=tk.LEFT, padx=6)

# ---- 3줄: 기타 ----
row3 = ttk.Frame(root)
row3.pack(fill=tk.X, padx=10, pady=4)
ttk.Button(row3, text="FT Zero", command=ft_zero).pack(side=tk.LEFT, padx=3)
ttk.Button(row3, text="Stop & Save / Plot", command=stop_and_save,
           style="Stop.TButton").pack(side=tk.LEFT, padx=6)
status_label = ttk.Label(row3, text="Ready  →  [1. Close Gripper] 부터 시작", foreground="#00ff88")
status_label.pack(side=tk.LEFT, padx=10)

# ---- 그래프 + 오른쪽 텍스트 ----
main_frame = ttk.Frame(root)
main_frame.pack(fill=tk.BOTH, expand=True)

plot_frame = ttk.Frame(main_frame)
plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

fig = Figure(figsize=(8, 7.5), dpi=100, facecolor="black")
ax_dcur = fig.add_subplot(511)
ax_dxl  = fig.add_subplot(512, sharex=ax_dcur)
ax_fz   = fig.add_subplot(513, sharex=ax_dcur)
ax_enc  = fig.add_subplot(514, sharex=ax_dcur)
ax_cur  = fig.add_subplot(515, sharex=ax_dcur)

# (축, 세로축 이름, [(버퍼, 선 모양), ...])
live_axes = [
    (ax_dcur, "DXL Cur [mA]",  [(buf_dxl_goal_cur, 'r--'), (buf_dxl_cur, 'm')]),
    (ax_dxl,  "DXL Enc [deg]", [(buf_dxl_deg, 'w')]),
    (ax_fz,   "FT Fz [N]",     [(buf_fz, '#4da6ff')]),
    (ax_enc,  "TSA Encoder",   [(buf_tsa_enc, 'c')]),
    (ax_cur,  "TSA Cur [A]",   [(buf_tsa_cur, 'g')]),
]
live_lines = []    # [(선, 버퍼), ...]
for ax, label, items in live_axes:
    ax.set_facecolor("black")
    ax.tick_params(colors="white", labelsize=8)
    for spine in ax.spines.values():
        spine.set_color("white")
    ax.grid(True, color="#444444", linestyle="--", alpha=0.5)
    ax.set_ylabel(label, color="white", fontsize=9)
    for buf, fmt_str in items:
        line, = ax.plot([], [], fmt_str, linewidth=1.5)
        live_lines.append((line, buf))
ax_cur.set_xlabel("Time [s]", color="white")
fig.subplots_adjust(left=0.11, right=0.97, top=0.98, bottom=0.07, hspace=0.25)

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

text_frame = ttk.Frame(main_frame, width=330)
text_frame.pack(side=tk.LEFT, fill=tk.Y)
text_frame.pack_propagate(False)

ttk.Label(text_frame, text="Live Data", font=("TkDefaultFont", 11, "bold")).pack(pady=5)
live_var = tk.StringVar()
ttk.Label(text_frame, textvariable=live_var, font=("TkFixedFont", 10),
          justify=tk.LEFT).pack(anchor="w", padx=5)


def fmt(v, unit=""):
    """아직 수신된 적 없는 값(NaN)은 '---' 로 표시"""
    if math.isnan(v):
        return "     --- (수신 없음)"
    return f"{v:10.3f} {unit}"


def connection_text():
    """연결 상태 진단 (오른쪽 패널 아래쪽)"""
    now = time.time()

    def alive(t):
        return "OK" if now - t < 0.5 else "수신 없음"

    if bus is None:
        can_line = "CAN         : 연결 안 됨 (터미널 확인)\n"
    else:
        can_line = f"CAN state   : {can_state_text}\n"
        if can_tx_errors:
            can_line += f"CAN TX 실패 : {can_tx_errors} 회\n"
    ids = sorted(can_rx_counts.items(), key=lambda kv: -kv[1])[:6]
    id_text = ", ".join(f"0x{i:03X}" for i, _ in ids) if ids else "없음"
    return (
        "---- 연결 상태 ----\n"
        + can_line
        + f"CAN 수신 ID : {id_text}\n"
        + f"FT 센서     : {alive(last_ft_time) if bus else '-'}\n"
        + f"TSA 피드백  : {alive(last_tsa_time) if bus else '-'}\n"
        + f"DXL         : {f'OK (ID {DXL_ID}, {DXL_PORT})' if dxl_ok else '연결 안 됨'}"
    )


def update_gui():
    """화면 갱신 (10 Hz): 상태 메시지, 버튼, 그래프, 텍스트"""
    global pending_status

    if not running:
        return

    if pending_status is not None:
        text, color = pending_status
        pending_status = None
        status_label.config(text=text, foreground=color)
    refresh_buttons()

    # 최근 구간만 복사해서 그림 (세로축은 자동으로 맞춰짐)
    n = int(TIME_WINDOW / RECORD_DT) + 50
    with buf_lock:
        t = buf_t[-n:]
        ys = [buf[-n:] for _, buf in live_lines]
        latest = [b[-1] if b else NAN for b in
                  (buf_dxl_deg, buf_dxl_tick, buf_dxl_goal_cur, buf_dxl_cur, buf_fz,
                   buf_tsa_enc, buf_tsa_cur, buf_ref_force, buf_out_force)]

    if len(t) >= 2:
        for (line, _), y in zip(live_lines, ys):
            line.set_data(t, y)

        t_end = t[-1]
        t_start = max(0.0, t_end - TIME_WINDOW)
        for ax, _, _ in live_axes:
            ax.set_xlim(t_start, t_start + TIME_WINDOW)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        dxl_deg, dxl_tick, goal_cur, dxl_cur, fz, enc, cur, ref_f, out_f = latest
        live_var.set(
            f"Time        : {t_end:10.3f} s\n\n"
            f"Phase       : {PHASE_NAME[phase]}\n\n"
            f"DXL Goal Cur: {fmt(goal_cur, 'mA')}\n"
            f"DXL Current : {fmt(dxl_cur, 'mA')}\n"
            f"DXL Encoder : {fmt(dxl_deg, 'deg')}\n"
            f"DXL Tick    : {fmt(dxl_tick)}\n\n"
            f"FT Fz       : {fmt(fz, 'N')}\n\n"
            f"TSA Encoder : {fmt(enc)}\n"
            f"TSA Current : {fmt(cur, 'A')}\n\n"
            f"TSA Ref F   : {fmt(ref_f, 'N')}\n"
            f"TSA Out F   : {fmt(out_f, 'N')}\n\n"
            + connection_text()
        )
        if recording:
            canvas.draw_idle()

    root.after(int(PLOT_DT * 1000), update_gui)


# =====================================================================
#  시작 / 종료
# =====================================================================
def on_close():
    global running
    if not running:
        return
    running = False
    try:
        send_force_value(0.0)
        if FT_ENABLE:
            ft_send_command(0x0C)      # FT 연속 출력 정지
    except Exception:
        pass
    if dxl_ok:
        with dxl_lock:
            dxl_torque(False)
            dxl_port.closePort()
    if bus is not None:
        try:
            bus.shutdown()
        except Exception:
            pass
    # can0 는 끄지 않고 그대로 둠 → 다음 실행 때 sudo 없이 바로 사용 가능
    root.destroy()
    sys.exit(0)


dxl_init()                             # 토크 OFF 상태로 연결만 함 (움직이지 않음)
if FT_ENABLE:
    ft_send_command(0x0B)              # FT 연속 출력 시작

if bus is not None:
    threading.Thread(target=can_rx_thread, daemon=True).start()
    threading.Thread(target=can_state_thread, daemon=True).start()
if dxl_ok:
    threading.Thread(target=dxl_thread, daemon=True).start()

if bus is None and not dxl_ok:
    set_status("CAN, 다이나믹셀 모두 연결 안 됨 (터미널 메시지 확인)", "red")
elif bus is None:
    set_status("CAN 연결 안 됨 - TSA/FT 없이 다이나믹셀만 동작 (터미널 메시지 확인)", "orange")
elif not dxl_ok:
    set_status("다이나믹셀 연결 안 됨 (터미널 메시지 확인) - TSA/FT 만 동작", "orange")

threading.Thread(target=logger_thread, daemon=True).start()
update_gui()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()