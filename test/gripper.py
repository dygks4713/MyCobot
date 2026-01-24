from pymycobot.mycobot import MyCobot
import time

# 1. 로봇 연결 및 라이브러리 로드
mc = MyCobot('COM4', 115200)   # 로봇 연결 설정 (포트: COM4, 보드레이트: 115200)

# 2. 그리퍼 초기화 설정
mc.set_gripper_mode(0)         # 그리퍼 동작 모드 설정 (0: 기본 제어 모드)
mc.init_eletric_gripper()      # 전동 그리퍼 초기화 수행
time.sleep(1)                  # 초기화 완료 대기를 위한 1초 정지

# 3. 메인 반복 루프 (그리퍼 반복 동작)
while True:
    # --- 그리퍼 닫기 동작 ---
    print("Gripper Close")     # 터미널에 닫힘 상태 출력
    mc.set_eletric_gripper(1)  # 전동 그리퍼 제어 활성화 (1: 닫기 방향)
    mc.set_gripper_value(100, 20) # 그리퍼 목표값 설정 (100: 완전히 닫힘, 속도: 20)
    time.sleep(2)              # 동작 완료 대기를 위한 2초 정지

    # --- 그리퍼 열기 동작 ---
    print("Gripper Open")      # 터미널에 열림 상태 출력
    mc.set_eletric_gripper(0)  # 전동 그리퍼 제어 활성화 (0: 열기 방향)
    mc.set_gripper_value(0, 20)   # 그리퍼 목표값 설정 (0: 완전히 열림, 속도: 20)
    time.sleep(2)              # 동작 완료 대기를 위한 2초 정지