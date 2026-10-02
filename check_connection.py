#!/usr/bin/env python3
"""TSA 역구동성 실험 - 하드웨어 연결 점검

1) CAN (can0): 꺼져 있으면 자동으로 bitrate 설정 후 켜고, 잠깐 수신 프레임을 확인
2) 시리얼 포트 목록
3) 다이나믹셀 검색 (FTDI/U2D2 포트만, Protocol 2.0 broadcast ping 우선)

사용법:
    python3 check_connection.py               # 기본 (can0, 1Mbps)
    python3 check_connection.py --no-can-up   # can0 자동 켜기 생략
    python3 check_connection.py --dxl-port /dev/ttyUSB0 --dxl-baud 3000000
"""

import argparse
import glob
import os
import shutil
import socket
import struct
import subprocess
import sys
import time


def header(title):
    print()
    print("=" * 60)
    print(title)
    print("=" * 60)


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


# ---------------------------------------------------------------------------
# 1) CAN
# ---------------------------------------------------------------------------
def can_is_up(iface):
    try:
        with open(f"/sys/class/net/{iface}/operstate") as f:
            state = f.read().strip()
        with open(f"/sys/class/net/{iface}/flags") as f:
            flags = int(f.read().strip(), 16)
    except FileNotFoundError:
        return None
    # IFF_UP = 0x1. CAN 인터페이스는 operstate 가 "unknown" 일 수도 있음
    return bool(flags & 0x1) and state in ("up", "unknown")


def can_bring_up(iface, bitrate):
    """can0 를 down -> bitrate/restart-ms 설정 -> up. 필요하면 sudo 사용."""
    prefix = [] if os.geteuid() == 0 else ["sudo"]
    if prefix and not shutil.which("sudo"):
        print("    sudo 가 없습니다. root 로 실행하세요.")
        return False
    cmds = [
        prefix + ["ip", "link", "set", iface, "down"],
        prefix + ["ip", "link", "set", iface, "type", "can",
                  "bitrate", str(bitrate), "restart-ms", "100"],
        prefix + ["ip", "link", "set", iface, "up"],
    ]
    for cmd in cmds:
        print("    $ " + " ".join(cmd))
        # sudo 비밀번호 입력을 받을 수 있도록 터미널에 그대로 연결
        r = subprocess.run(cmd)
        if r.returncode != 0:
            print(f"    [X] 실패 (exit {r.returncode})")
            return False
    time.sleep(0.2)
    return True


def can_listen(iface, seconds):
    """raw SocketCAN 으로 잠깐 수신해 버스에 다른 노드가 있는지 확인."""
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
    except OSError as e:
        print(f"    소켓 열기 실패: {e}")
        return
    s.settimeout(0.1)
    # RFT40 FT 센서는 연속 출력 시작 명령(ID 0x64, 0x0B)을 받아야 데이터(0x01, 0x02)를 보냄
    try:
        s.send(struct.pack("=IB3x8s", 0x64, 8, bytes([0x0B, 0, 0, 0, 0, 0, 0, 0])))
    except OSError as e:
        print(f"    FT 시작 명령 전송 실패: {e}")
    frames = {}
    end = time.time() + seconds
    while time.time() < end:
        try:
            data = s.recv(16)
        except socket.timeout:
            continue
        can_id, dlc = struct.unpack("=IB3x", data[:8])
        can_id &= socket.CAN_EFF_MASK
        frames.setdefault(can_id, [0, data[8:8 + dlc]])
        frames[can_id][0] += 1
    try:
        s.send(struct.pack("=IB3x8s", 0x64, 8, bytes([0x0C, 0, 0, 0, 0, 0, 0, 0])))  # FT 출력 정지
    except OSError:
        pass
    s.close()

    if 1 in frames and 2 in frames:
        print("    [O] FT 센서(RFT40) 응답 확인 (ID 0x001 + 0x002)")
    if not frames:
        print(f"    {seconds:.0f}초 동안 수신된 CAN 프레임 없음")
        print("    (모터가 요청을 받아야만 응답하는 타입이면 정상일 수 있음."
              " 전원/배선/종단저항 120Ω 확인)")
        return
    print(f"    {seconds:.0f}초 동안 수신된 ID:")
    for cid in sorted(frames):
        cnt, payload = frames[cid]
        print(f"      0x{cid:03X}  x{cnt:<5d} 예: {payload.hex(' ')}")


def check_can(iface, bitrate, auto_up, listen_sec):
    header(f"1) CAN ({iface})")
    if not os.path.exists(f"/sys/class/net/{iface}"):
        print(f"[X] {iface} 가 없습니다. PCAN-USB 연결 / 드라이버(peak_usb) 확인")
        return False

    up = can_is_up(iface)
    if not up:
        if not auto_up:
            print(f"[X] {iface} 가 꺼져 있습니다. 아래 명령 후 다시 실행하세요.")
            print(f"    sudo ip link set {iface} up type can bitrate {bitrate}")
            return False
        print(f"[!] {iface} 가 꺼져 있어 켭니다 (bitrate {bitrate})")
        if not can_bring_up(iface, bitrate) or not can_is_up(iface):
            print(f"[X] {iface} 를 켜지 못했습니다.")
            return False

    detail = run(["ip", "-details", "-statistics", "link", "show", iface]).stdout
    state = "?"
    for line in detail.splitlines():
        line = line.strip()
        if line.startswith("can state"):
            state = line.split()[2]
    print(f"[O] {iface} UP, can state = {state}")
    if state in ("BUS-OFF", "ERROR-PASSIVE"):
        print("    버스 에러 상태입니다. bitrate 불일치, 종단저항, CAN_H/L 배선을 확인하세요.")
    can_listen(iface, listen_sec)
    return True


# ---------------------------------------------------------------------------
# 2) 시리얼 포트
# ---------------------------------------------------------------------------
def list_serial_ports():
    header("2) 시리얼 포트")
    try:
        from serial.tools import list_ports
        ports = [(p.device, p.description, p.manufacturer or "", p.vid, p.pid)
                 for p in list_ports.comports()]
    except ImportError:
        print("(pyserial 미설치: pip install pyserial)")
        ports = [(d, "", "", None, None)
                 for d in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))]

    ports = [p for p in ports if "ttyUSB" in p[0] or "ttyACM" in p[0]]
    if not ports:
        print("[X] USB 시리얼 포트가 없습니다.")
    for dev, desc, manu, vid, pid in ports:
        ok = os.access(dev, os.R_OK | os.W_OK)
        vp = f"VID:PID={vid:04x}:{pid:04x}" if vid is not None else ""
        perm = "읽기/쓰기 가능" if ok else "권한 없음 (sudo usermod -aG dialout $USER 후 재로그인)"
        print(f"{dev}  -  {desc} | {manu} | {vp}  -  {perm}")
    return ports


# ---------------------------------------------------------------------------
# 3) 다이나믹셀
# ---------------------------------------------------------------------------
DXL_BAUDS = [57600, 1000000, 3000000, 4000000, 115200, 2000000, 4500000, 9600]
ARDUINO_VID = 0x2341

MODEL_NAMES = {
    311: "MX-64(2.0)", 321: "MX-106(2.0)", 30: "MX-28(2.0)",
    1020: "XM430-W350", 1030: "XM430-W210", 1120: "XM540-W270",
    1130: "XM540-W150", 1060: "XL430-W250", 1200: "XL330-M288",
    1190: "XL330-M077", 1010: "XH430-W210", 1000: "XH430-W350",
}


def dxl_scan_port(dxl, port, bauds):
    ph = dxl.PortHandler(port)
    if not ph.openPort():
        print(f"  [X] {port} 열기 실패")
        return []
    found = []
    try:
        for baud in bauds:
            if not ph.setBaudRate(baud):
                continue
            # Protocol 2.0: broadcast ping 한 번으로 전체 ID 검색 (빠름)
            pk2 = dxl.PacketHandler(2.0)
            res, comm = pk2.broadcastPing(ph)
            if comm == dxl.COMM_SUCCESS and res:
                for dxl_id, (model, fw) in sorted(res.items()):
                    found.append((2.0, baud, dxl_id, model, fw))
                    name = MODEL_NAMES.get(model, "?")
                    print(f"  [O] Protocol 2.0 | baud {baud} | ID {dxl_id} | "
                          f"model {model} ({name}) | firmware {fw}")
                break  # 찾았으면 다른 baud 는 생략
            # Protocol 1.0: broadcast ping 이 없으므로 ID 0~30 만 빠르게 확인
            pk1 = dxl.PacketHandler(1.0)
            for dxl_id in range(0, 31):
                model, comm, err = pk1.ping(ph, dxl_id)
                if comm == dxl.COMM_SUCCESS:
                    found.append((1.0, baud, dxl_id, model, None))
                    print(f"  [O] Protocol 1.0 | baud {baud} | ID {dxl_id} | model {model}")
            if found:
                break
    finally:
        ph.closePort()
    return found


def check_dynamixel(ports, only_port=None, only_baud=None):
    header("3) 다이나믹셀 검색")
    try:
        import dynamixel_sdk as dxl
    except ImportError:
        print("[X] dynamixel_sdk 미설치: pip install dynamixel-sdk")
        return []

    bauds = [only_baud] if only_baud else DXL_BAUDS
    if only_port:
        targets = [only_port]
    else:
        targets = []
        for dev, desc, manu, vid, pid in ports:
            # Arduino(Due 등)는 다이나믹셀이 아님. 열기만 해도 리셋될 수 있어 제외
            if vid == ARDUINO_VID or "arduino" in (desc + manu).lower():
                print(f"{dev}: Arduino 포트라 건너뜀")
                continue
            targets.append(dev)

    found = []
    for port in targets:
        print(f"{port}: 검색 중... (baud {len(bauds)}개)")
        r = dxl_scan_port(dxl, port, bauds)
        if not r:
            print(f"  [X] {port} 에서 다이나믹셀 없음 (전원 12V, 케이블, U2D2 확인)")
        found += r
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can", default="can0")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--no-can-up", action="store_true", help="can0 자동 켜기 안 함")
    ap.add_argument("--listen", type=float, default=2.0, help="CAN 수신 확인 시간(초)")
    ap.add_argument("--dxl-port", help="이 포트만 검색 (예: /dev/ttyUSB0)")
    ap.add_argument("--dxl-baud", type=int, help="이 baud 만 검색 (예: 3000000)")
    args = ap.parse_args()

    can_ok = check_can(args.can, args.bitrate, not args.no_can_up, args.listen)
    ports = list_serial_ports()
    dxls = check_dynamixel(ports, args.dxl_port, args.dxl_baud)

    header("요약")
    print(f"CAN ({args.can})  : {'OK' if can_ok else '실패'}")
    print(f"시리얼 포트       : {len(ports)}개")
    if dxls:
        for proto, baud, dxl_id, model, _ in dxls:
            print(f"다이나믹셀        : ID {dxl_id}, {MODEL_NAMES.get(model, model)}, "
                  f"Protocol {proto}, baud {baud}")
    else:
        print("다이나믹셀        : 없음")
    return 0 if (can_ok and dxls) else 1


if __name__ == "__main__":
    sys.exit(main())
