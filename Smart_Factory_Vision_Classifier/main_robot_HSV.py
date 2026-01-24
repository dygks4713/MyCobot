import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# 1. 로봇 및 상수 설정
mc = MyCobot('COM10', 115200) # 로봇 포트 설정
BLOCK_HEIGHT = 25.0  # 블록 높이 (mm)
stack_count = 0      # 정상품 적재 카운트

# 2. 공정 데이터 정의 (기본 위치 단계 제거, 총 8단계)
process_steps = [
    {"name": "1. 물건 위에 있기 및 색깔 인식", "coords": [229.7, 193.8, 261.3, -162.0, 6.39, -90.77], "grip": 40},
    {"name": "2. 물건 위에 있기", "coords": [222.7, 193.8, 250.3, -177.0, 5.39, -87.77], "grip": 40},
    {"name": "3. 물건 잡기", "coords": [222.7, 193.8, 190.3, -177.0, 5.39, -87.77], "grip": 10},
    {"name": "4. 물건 들어올리기", "coords": [222.7, 193.8, 250.3, -177.0, 5.39, -87.77], "grip": 10},
    {"name": "5. 이동하기", "coords": [206.7, -235.2, 248.3, -177.39, 2.39, -87.77], "grip": 10},
    {"name": "6. 물건 놓기", "coords": [206.7, -235.2, 181.3, -177.39, 2.39, -87.77], "grip": 40},
    {"name": "7. 위로 살짝 올리기", "coords": [206.7, -235.2, 248.3, -177.39, 2.39, -87.77], "grip": 40},
    {"name": "8. 기본 위치 복귀", "coords": [244.70, -86.20, 306.30, -178.39, 1.39, -87.77], "grip": 40},
]

# 3. 불량 위치 좌표 (빨간색 전용)
red_defective_coords = {
    "이동하기": [70.7, -249.2, 306.3, -177.39, 2.39, -92.77],
    "물건 놓기": [70.7, -249.2, 182.3, -177.39, 2.39, -92.77],
    "위로 살짝 올리기": [70.7, -249.2, 306.3, -177.39, 2.39, -92.77]
}

# HSV 색상 범위 (기존 유지)
color_ranges = {
    "Red": [(np.array([0, 150, 50]), np.array([10, 255, 255])), (np.array([160, 150, 50]), np.array([180, 255, 255]))],
    "Green": [(np.array([35, 100, 50]), np.array([85, 255, 255]))],
    "Blue": [(np.array([90, 100, 50]), np.array([130, 255, 255]))],
    "Purple": [(np.array([120, 50, 40]), np.array([160, 255, 255]))]
}

def detect_color_4s():
    """ 4초 연속 동일 색상 감지 로직 """
    cap = cv2.VideoCapture(0)
    last_color, start_time = None, 0
    while True:
        ret, frame = cap.read()
        if not ret: break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_now = "None"
        for color_name, ranges in color_ranges.items():
            mask = None
            for (low, high) in ranges:
                m = cv2.inRange(hsv, low, high)
                mask = m if mask is None else cv2.add(mask, m)
            if cv2.countNonZero(mask) > 1000:
                detected_now = color_name; break
        if detected_now != "None":
            if detected_now == last_color:
                elapsed = time.time() - start_time
                cv2.putText(frame, f"{detected_now}: {elapsed:.1f}s", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                if elapsed >= 4:
                    cap.release(); cv2.destroyAllWindows(); return detected_now
            else:
                last_color = detected_now; start_time = time.time()
        else:
            last_color = None; start_time = 0
        cv2.imshow('Color Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    cap.release(); cv2.destroyAllWindows(); return None

def move_robot(name, coords, grip):
    """ 로봇 이동 및 그리퍼 제어 (명사종결형 주석) """
    print(f"\n[실행] {name} 이동")
    mc.send_coords(coords, 20, 0)  # 직선 이동 모드
    time.sleep(3)                 # 이동 대기
    mc.set_gripper_value(grip, 20) # 그리퍼 작동
    time.sleep(2)                 # 동작 대기

def run_process():
    global stack_count
    
    # 1. 첫 번째 단계(색깔 인식 위치)로 이동 및 스캔
    move_robot(process_steps[0]["name"], process_steps[0]["coords"], process_steps[0]["grip"])
    color = detect_color_4s()
    if not color: return

    print(f"\n>>> 감지된 색상: {color}")
    is_red = (color == "Red")

    # 2. 두 번째 단계(인덱스 1)부터 나머지 공정 수행
    for i in range(1, len(process_steps)):
        step = process_steps[i]
        current_coords = list(step["coords"])
        current_name = step["name"]

        if is_red:
            # 빨간색일 경우 불량 위치 좌표로 교체
            if "이동하기" in current_name: current_coords = red_defective_coords["이동하기"]
            elif "물건 놓기" in current_name: current_coords = red_defective_coords["물건 놓기"]
            elif "위로 살짝 올리기" in current_name: current_coords = red_defective_coords["위로 살짝 올리기"]
        else:
            # 정상품일 경우 적재 높이 보정 (6번: 물건 놓기 단계 - 인덱스 5)
            if i == 5: 
                current_coords[2] += (stack_count * BLOCK_HEIGHT)

        move_robot(current_name, current_coords, step["grip"])
    
    # 결과 출력 및 카운트 관리
    if not is_red:
        stack_count += 1
        print(f"정상품 적재 완료 (현재: {stack_count}개)")
    else:
        print("불량 위치 배출 완료")

# --- 메인 실행부 ---

# 그리퍼 및 서보 초기화
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(3)

# 프로그램 시작 시 바로 1단계(인식 위치)로 이동
print("\n[초기화] 공정 시작 위치로 이동합니다.")
move_robot(process_steps[7]["name"], process_steps[7]["coords"], process_steps[7]["grip"])

while True:
    print(f"\n[현재 적재수: {stack_count}] S: 공정 시작 | R: 카운트 리셋 | Q: 프로그램 종료")
    user_input = input(">> ").upper()
    if user_input == 'S':
        run_process()
    elif user_input == 'R':
        stack_count = 0
        print("적재 카운트가 리셋되었습니다.")
    elif user_input == 'Q':
        print("프로그램을 종료합니다.")
        break