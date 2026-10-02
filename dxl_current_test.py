#!/usr/bin/env python3
"""다이나믹셀 전류 센서 확인용 테스트

토크 OFF 상태에서는 모터에 전류가 흐르지 않으므로 손으로 돌려도 Present Current 는 0 근처가 정상.
이 스크립트는 '전류 기반 위치제어(mode 5)' 로 현재 위치를 붙잡고 있게 한 뒤,
손으로 밀면 버티느라 전류가 올라가는지 출력한다. (전류 상한 HOLD_CURRENT_MA 로 제한)

    python3 dxl_current_test.py            # 10초 동안 측정
    python3 dxl_current_test.py 30         # 30초 동안 측정
"""

import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DXL_PORT = '/dev/ttyUSB0'
DXL_BAUD = 3000000
DXL_ID = 1

HOLD_CURRENT_MA = 300.0   # 붙잡는 힘의 상한 [mA]

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126   # 126~135: 전류 2 + 속도 4 + 위치 4
MODE_CURRENT_BASED_POSITION = 5
CURRENT_UNIT_MA = 3.36


def check(result, what):
    res, err = result
    if res != COMM_SUCCESS:
        sys.exit(f"[X] {what} 실패: {packet.getTxRxResult(res)}")
    if err != 0:
        print(f"[!] {what}: {packet.getRxPacketError(err)}")


def read_state():
    data, res, _ = packet.readTxRx(port, DXL_ID, ADDR_PRESENT_CURRENT, 10)
    if res != COMM_SUCCESS:
        return None
    cur = int.from_bytes(bytes(data[0:2]), 'little', signed=True) * CURRENT_UNIT_MA
    pos = int.from_bytes(bytes(data[6:10]), 'little', signed=True)
    return cur, pos


duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

port = PortHandler(DXL_PORT)
packet = PacketHandler(2.0)
if not port.openPort() or not port.setBaudRate(DXL_BAUD):
    sys.exit(f"[X] {DXL_PORT} 를 열 수 없습니다")

try:
    # 1) 토크 OFF 상태 전류 (0 근처면 정상)
    check(packet.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, 0), "토크 OFF")
    s = read_state()
    print(f"토크 OFF 상태 전류: {s[0]:.1f} mA  (0 근처가 정상)")

    # 2) 현재 위치를 붙잡게 함
    check(packet.write1ByteTxRx(port, DXL_ID, ADDR_OPERATING_MODE,
                                MODE_CURRENT_BASED_POSITION), "모드 변경")
    check(packet.write2ByteTxRx(port, DXL_ID, ADDR_GOAL_CURRENT,
                                int(HOLD_CURRENT_MA / CURRENT_UNIT_MA)), "전류 상한 설정")
    pos = read_state()[1]
    check(packet.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_POSITION, pos & 0xFFFFFFFF), "목표 위치")
    check(packet.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, 1), "토크 ON")

    print(f"\n토크 ON (위치 {pos} 붙잡는 중, 전류 상한 {HOLD_CURRENT_MA:.0f} mA)")
    print(f"지금 손으로 모터를 밀어 보세요. {duration:.0f}초 동안 측정합니다.\n")
    t_end = time.time() + duration
    peak = 0.0
    while time.time() < t_end:
        s = read_state()
        if s:
            cur, p = s
            peak = max(peak, abs(cur))
            bar = "#" * min(40, int(abs(cur) / 10))
            print(f"\r전류 {cur:8.1f} mA | 위치 {p:8d} | {bar:<40}", end="", flush=True)
        time.sleep(0.05)
    print(f"\n\n최대 |전류| = {peak:.1f} mA")
    print("→ 밀 때 전류가 올라갔으면 센서/코드 정상." if peak > 20
          else "→ 밀어도 전류가 안 변하면 모터 전원(12V) / 모델 / 펌웨어 확인 필요.")
finally:
    packet.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    port.closePort()
    print("토크 OFF, 종료")
