#!/usr/bin/env python3
"""TSA (Arduino Due, USB 시리얼) 통신 진단 - Due 가 실제로 무엇을 보내는지 그대로 보여줌

    python3 tsa_test.py                 # Due 포트 자동, 115200 bps, 5초 수신
    python3 tsa_test.py --baud 9600
    python3 tsa_test.py --scan          # 글자가 깨지면 여러 baud 로 시도
    python3 tsa_test.py --send "F0"     # 명령을 보내고 응답 확인 (줄바꿈 자동 추가)

※ Arduino IDE 시리얼 모니터는 닫고 실행하세요.
"""

import argparse
import glob
import os
import sys
import time

import serial
from serial.tools import list_ports

ARDUINO_VID = 0x2341
SCAN_BAUDS = [115200, 9600, 57600, 230400, 250000, 460800, 921600, 1000000, 2000000]


def find_due_ports():
    ports = [p for p in list_ports.comports() if p.vid == ARDUINO_VID]
    for p in ports:
        kind = {0x003d: "Programming Port", 0x003e: "Native USB Port"}.get(p.pid, "Arduino")
        print(f"  {p.device}: {p.description} ({kind}, PID {p.pid:04x})")
    return ports


def users_of(dev):
    """이 포트를 열고 있는 다른 프로세스 (Arduino IDE 시리얼 모니터 등)"""
    real = os.path.realpath(dev)
    users = []
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.path.realpath(fd) == real:
                pid = fd.split("/")[2]
                with open(f"/proc/{pid}/comm") as f:
                    users.append(f"{f.read().strip()} (pid {pid})")
        except OSError:
            continue
    return sorted(set(users))


def capture(dev, baud, seconds, send=None):
    try:
        ser = serial.Serial(dev, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"  [X] 열기 실패: {e}")
        return None
    time.sleep(2.0)                      # Programming Port 는 열면 Due 가 리셋됨 → 부팅 대기
    if send:
        ser.reset_input_buffer()
        ser.write((send + "\n").encode())
        print(f"  보냄: {send!r}")
    data = b""
    end = time.time() + seconds
    while time.time() < end:
        data += ser.read(4096)
    ser.close()
    return data


def show(data):
    if not data:
        print("  수신 0 byte")
        return 0.0
    printable = sum(32 <= b < 127 or b in (9, 10, 13) for b in data) / len(data)
    lines = data.decode(errors="replace").splitlines()
    print(f"  수신 {len(data)} byte, {len(lines)} 줄, 정상 글자 비율 {printable * 100:.0f}%")
    for line in lines[:15]:
        print(f"    {line!r}")
    if printable < 0.9:
        print(f"  (hex 앞부분) {data[:48].hex(' ')}")
    return printable


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--time", type=float, default=5.0)
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--send")
    args = ap.parse_args()

    print("Arduino 포트:")
    ports = find_due_ports()
    dev = args.port or (sorted(ports, key=lambda p: p.pid != 0x003e)[0].device if ports else None)
    if not dev:
        sys.exit("[X] Arduino Due 포트 없음 → USB 케이블/포트 확인 (데이터 케이블인지도)")
    if not os.access(dev, os.R_OK | os.W_OK):
        sys.exit(f"[X] {dev} 권한 없음 → sudo usermod -aG dialout $USER 후 재로그인")
    users = users_of(dev)
    if users:
        print(f"[!] {dev} 를 다른 프로그램이 사용 중: {', '.join(users)} → 닫고 다시 실행")

    bauds = SCAN_BAUDS if args.scan else [args.baud]
    for baud in bauds:
        print(f"\n[{dev} @ {baud} bps, {args.time:.0f}초 수신]")
        data = capture(dev, baud, args.time, args.send)
        if data is None:
            return
        ok = show(data)
        if data and ok >= 0.9:
            print(f"\n[O] 정상 수신 ({baud} bps). 위 줄 형식을 GUI 의 TSA_FIELDS 에 맞추면 됩니다.")
            return
        if not data and not args.scan:
            break

    print()
    if args.scan:
        print("[X] 어떤 baud 에서도 정상 글자가 안 나옴")
    print("확인할 것:")
    print("  1) 펌웨어가 이 포트로 출력하는지: Programming Port → Serial.print, "
          "Native Port → SerialUSB.print")
    print("  2) Serial.begin(...) 값과 --baud 값이 같은지 (글자가 깨지면 불일치) → --scan")
    print("  3) 펌웨어가 명령을 받아야만 출력하는 구조인지 → --send \"명령\" 으로 확인")
    print("  4) 전압/전류 값은 펌웨어가 Serial 로 보내야만 PC 에서 읽을 수 있음")


if __name__ == "__main__":
    main()
