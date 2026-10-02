#!/usr/bin/env python3
"""FT 센서 (Robotous RFT40-SA01, CAN) 통신 진단

    python3 ft_test.py           # 1 Mbps 로 점검
    python3 ft_test.py --scan    # 여러 bitrate 로 시도 (센서 bitrate 를 모를 때)

순서: can0 설정 확인/수정 → 모델명 읽기(0x01) → 연속 출력(0x0B) 2초 수신 → 원인 진단
"""

import argparse
import os
import socket
import struct
import subprocess
import sys
import time

IFACE = "can0"
FT_CMD_ID = 0x64
FT_RESP_IDS = (0x01, 0x02)
FORCE_DIV, TORQUE_DIV = 50.0, 2000.0
SCAN_BITRATES = [1000000, 500000, 250000, 125000]


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout


def can_info():
    out = sh(f"ip -details link show {IFACE}")
    words = out.split()
    info = {"up": ",UP" in out or "<UP" in out, "bitrate": None, "restart_ms": None,
            "state": "?", "tx_err": 0, "rx_err": 0}
    for i, w in enumerate(words[:-1]):
        if w == "bitrate" and words[i + 1].isdigit():
            info["bitrate"] = int(words[i + 1])
        elif w == "restart-ms" and words[i + 1].isdigit():
            info["restart_ms"] = int(words[i + 1])
        elif w == "state" and words[i - 1] == "can":
            info["state"] = words[i + 1]
        elif w.endswith("berr-counter") and i + 4 < len(words):
            info["tx_err"] = int(words[i + 2])
            info["rx_err"] = int(words[i + 4].rstrip(")"))
    return info


def stat(name):
    try:
        with open(f"/sys/class/net/{IFACE}/statistics/{name}") as f:
            return int(f.read())
    except OSError:
        return 0


def configure(bitrate):
    print(f"  can0 설정: bitrate {bitrate}, restart-ms 100  (sudo 비밀번호를 물을 수 있음)")
    for c in (f"sudo ip link set {IFACE} down",
              f"sudo ip link set {IFACE} type can bitrate {bitrate} restart-ms 100",
              f"sudo ip link set {IFACE} up"):
        if subprocess.run(c, shell=True).returncode != 0:
            sys.exit(f"[X] 실패: {c}")
    time.sleep(0.2)


def open_socket():
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((IFACE,))
    s.settimeout(0.05)
    return s


def send(s, cmd, arg=0):
    try:
        s.send(struct.pack("=IB3x8s", FT_CMD_ID, 8, bytes([cmd, arg, 0, 0, 0, 0, 0, 0])))
        return True
    except OSError as e:
        print(f"  [X] 전송 실패: {e}  (보통 아무도 ACK 를 안 해서 송신 버퍼가 가득 찬 경우)")
        return False


def listen(s, seconds):
    """seconds 동안 수신 → [(id, bytes), ...]"""
    frames = []
    end = time.time() + seconds
    while time.time() < end:
        try:
            data = s.recv(16)
        except socket.timeout:
            continue
        can_id, dlc = struct.unpack("=IB3x", data[:8])
        if can_id & socket.CAN_ERR_FLAG:
            continue
        frames.append((can_id & socket.CAN_EFF_MASK, data[8:8 + dlc]))
    return frames


def pair_ft(frames):
    """0x01 + 0x02 를 짝지어 16 byte 패킷 목록으로"""
    packets, first = [], None
    for cid, d in frames:
        if cid == 0x01 and len(d) == 8:
            first = d
        elif cid == 0x02 and len(d) == 8 and first is not None:
            packets.append(first + d)
            first = None
    return packets


def try_bitrate(bitrate, verbose=True):
    info = can_info()
    if not info["up"] or info["bitrate"] != bitrate or not info["restart_ms"] \
            or info["state"] == "BUS-OFF":
        configure(bitrate)
    s = open_socket()
    listen(s, 0.1)                       # 남아 있던 프레임 비우기

    before = {k: stat(k) for k in ("tx_packets", "rx_packets", "tx_errors", "rx_errors")}

    # 1) 모델명 읽기
    if send(s, 0x01):
        pk = pair_ft(listen(s, 0.5))
        if pk and verbose:
            name = pk[0][1:].split(b"\0")[0].decode(errors="replace").strip()
            print(f"  [O] 모델명 응답: {name!r}")

    # 2) 연속 출력 2초
    send(s, 0x0B)
    frames = listen(s, 2.0)
    send(s, 0x0C)
    s.close()

    after = {k: stat(k) for k in before}
    info = can_info()
    data_pk = [p for p in pair_ft(frames) if p[0] in (0x0A, 0x0B)]
    others = sorted({cid for cid, _ in frames if cid not in FT_RESP_IDS})

    if verbose:
        print(f"  can state {info['state']}, 에러카운터 tx {info['tx_err']} / rx {info['rx_err']}")
        print(f"  송신 성공 {after['tx_packets'] - before['tx_packets']} 개, "
              f"수신 {after['rx_packets'] - before['rx_packets']} 개, "
              f"FT 데이터 {len(data_pk)} 개 (약 {len(data_pk) / 2:.0f} Hz)")
        if others:
            print(f"  FT 가 아닌 수신 ID: {', '.join(f'0x{c:03X}' for c in others)}")
        if data_pk:
            raw = struct.unpack(">6h", data_pk[-1][1:13])
            ft = [r / FORCE_DIV for r in raw[:3]] + [r / TORQUE_DIV for r in raw[3:]]
            print("  최근 값: Fx {:.2f} Fy {:.2f} Fz {:.2f} N | Tx {:.3f} Ty {:.3f} Tz {:.3f} Nm"
                  .format(*ft))
    return data_pk, frames, info, after["tx_packets"] - before["tx_packets"]


def diagnose(data_pk, frames, info, tx_ok):
    print()
    if data_pk:
        print("[O] FT 센서 통신 정상. GUI 에서도 받아져야 합니다.")
        return
    if frames:
        print("[!] CAN 프레임은 오는데 FT 형식(0x01+0x02, 첫 바이트 0x0B)이 아닙니다.")
        print("    → 센서의 CAN ID 가 바뀌었을 수 있음. 위 '수신 ID' 를 알려주세요.")
        return
    if tx_ok == 0 or info["tx_err"] >= 96 or info["state"] in ("ERROR-PASSIVE", "BUS-OFF"):
        print("[X] PC 가 보낸 명령을 아무도 받지(ACK) 않았습니다. = 버스에 FT 센서가 안 보임")
        print("    하드웨어/설정 문제입니다. 아래 순서로 확인하세요:")
        print("    1) 센서 전원: 전원선(VCC/GND) 연결, 전압이 매뉴얼 사양과 맞는지 (멀티미터로)")
        print("    2) 배선: CAN_H ↔ CAN_H, CAN_L ↔ CAN_L (서로 바뀌면 안 됨), GND 공통 연결")
        print("    3) 종단저항: 전원 끄고 CAN_H-CAN_L 사이 저항 측정 → 약 60Ω 이어야 정상")
        print("       120Ω 이면 한쪽 끝만, 무한대면 둘 다 없음 → 120Ω 저항 추가")
        print("    4) bitrate: 센서 설정이 1 Mbps 가 아닐 수 있음 → python3 ft_test.py --scan")
        print("    5) 센서가 CAN 이 아니라 UART 모드/UART 버전인지 (Robotous 설정 프로그램으로 확인)")
    elif info["rx_err"] > 0:
        print("[X] 수신 에러가 있습니다 → bitrate 불일치 또는 배선/노이즈 문제")
        print("    python3 ft_test.py --scan 으로 bitrate 를 찾아보세요.")
    else:
        print("[!] 명령은 전달(ACK)됐는데 FT 센서가 응답하지 않습니다.")
        print("    → 센서의 명령 ID 가 0x64 가 아니거나, 다른 장치만 ACK 하는 중일 수 있습니다.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan", action="store_true", help="여러 bitrate 로 시도")
    args = ap.parse_args()

    if not os.path.exists(f"/sys/class/net/{IFACE}"):
        sys.exit(f"[X] {IFACE} 없음: PCAN-USB 연결 확인 (lsusb, dmesg | grep -i peak)")

    info = can_info()
    print(f"현재 {IFACE}: {'UP' if info['up'] else 'DOWN'}, bitrate {info['bitrate']}, "
          f"restart-ms {info['restart_ms']}, state {info['state']}")
    if info["restart_ms"] == 0:
        print("  [!] restart-ms 0 → 한 번 BUS-OFF 되면 다시 켤 때까지 통신이 영원히 멈춤 (수정함)")

    if not args.scan:
        print("\n[1 Mbps 로 FT 센서 점검]")
        diagnose(*try_bitrate(1000000))
        return

    found = None
    for br in SCAN_BITRATES:
        print(f"\n[{br} bps 시도]")
        data_pk, *_ = try_bitrate(br)
        if data_pk:
            found = br
            break
    print()
    if found:
        print(f"[O] FT 센서 bitrate = {found} bps")
        if found != 1000000:
            print(f"    GUI 의 CAN_BITRATE = {found} 로 바꾸세요. (can0 는 지금 {found} 로 설정되어 있음)")
    else:
        print("[X] 어떤 bitrate 에서도 응답 없음 → 전원 / 배선 / 종단저항 / CAN 모드 문제")
        configure(1000000)


if __name__ == "__main__":
    main()
