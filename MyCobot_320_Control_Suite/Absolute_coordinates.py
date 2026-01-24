import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
import time

# ==========================
# 1) MyCobot 연결 및 초기화
# ==========================
mc = MyCobot("COM10", 115200) # 로봇 포트 설정
time.sleep(1)

# 초기화 명령 (루프 밖에서 한 번만 실행)
print("초기화")
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(1)

print("기본 위치 이동")
home_angles = [0, 0, 0, 0, 0, 0]
mc.send_angles(home_angles, 50)
# mc.set_gripper_value(0, 50) # 시작 시 열기
time.sleep(2)

win_name = "MyCobot Controller"
cv2.namedWindow(win_name)

print("-" * 30)
print("절대 좌표 제어 가이드:")
print("- 'c' 키: 목표 좌표(X, Y, Z, Rx, Ry, Rz) 입력")
print("- 'g' 키: 그리퍼 값(0~100) 입력")
print("- 'q' 키: 프로그램 종료")
print("-" * 30)

def nothing(x):
    pass

# ---------------------------------------------------------
# 트랙바 중앙 정렬을 위한 공백(padding) 추가
# ---------------------------------------------------------
pad = ' ' * 15  # 공백 개수를 조절하여 위치를 맞춥니다.

# 트랙바 생성 (J1~J6)
for i in range(1, 7):
    cv2.createTrackbar(f"J{i}", win_name, 180, 360, nothing)
# 그리퍼 트랙바
cv2.createTrackbar("Grip", win_name, 0, 100, nothing)

# 이전 값을 저장할 변수
prev_angles = [0, 0, 0, 0, 0, 0]
prev_grip = 0

# ==========================
# 2) 메인 루프
# ==========================
while True:
    # 트랙바 값 읽기
    curr_angles = []
    for i in range(1, 7):
        curr_angles.append(cv2.getTrackbarPos(f"J{i}", win_name) - 180)
    curr_grip = cv2.getTrackbarPos("Grip", win_name)

    # 값이 변했을 때만 로봇에 명령 전송 (통신 부하 방지)
    if curr_angles != prev_angles:
        mc.send_angles(curr_angles, 80)
        prev_angles = curr_angles.copy()
        time.sleep(0.02)

    if curr_grip != prev_grip:
        mc.set_gripper_value(curr_grip, 50)
        prev_grip = curr_grip
        time.sleep(0.02)

    # 데이터 피드백 (좌표 및 각도 수신)
    coords = mc.get_coords()
    real_angles = mc.get_angles()

    img = np.zeros((360, 800, 3), np.uint8)
    
    cv2.putText(img, "JOINT & GRIPPER MONITOR", (30, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if coords and len(coords) == 6:
        # 1. 위치 정보 (X, Y, Z) - 초록색
        cv2.putText(img, "[ Position (mm) ]", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)
        cv2.putText(img, f"X: {coords[0]:.2f}  Y: {coords[1]:.2f}  Z: {coords[2]:.2f}", 
                    (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 2. 자세 정보 (Rx, Ry, Rz) - 하늘색
        cv2.putText(img, "[ Orientation (deg) ]", (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)
        cv2.putText(img, f"Rx: {coords[3]:.2f}  Ry: {coords[4]:.2f}  Rz: {coords[5]:.2f}", 
                    (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 3. 그리퍼 정보 - 노란색
        cv2.putText(img, f"Grip: {curr_grip}%", (30, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 4. 각 조인트 각도 (참고용)
        # 위치를 조금 위로 올려서 전체적인 균형을 맞춤
        cv2.putText(img, "[ Joint Angles ]", (400, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        for i, ang in enumerate(real_angles):
            cv2.putText(img, f"J{i+1}: {ang:.1f}", (420, 120 + i*25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    cv2.imshow(win_name, img)

    key = cv2.waitKey(1) & 0xFF
    
    # --- 좌표 입력 모드 ---
    if key == ord('c'):
        print("\n[목표 좌표 입력 모드]")
        try:
            # 한 번에 입력받거나 하나씩 입력받기
            new_x = float(input("Target X: "))
            new_y = float(input("Target Y: "))
            new_z = float(input("Target Z: "))
            new_rx = float(input("Target Rx (기본 -180): "))
            new_ry = float(input("Target Ry (기본 0): "))
            new_rz = float(input("Target Rz (기본 0): "))
            
            target = [new_x, new_y, new_z, new_rx, new_ry, new_rz]
            print(f"Moving to: {target}")
            
            # send_coords(coords, speed, mode)
            # mode 0: 각 관절이 동시에 움직임 (빠름)
            # mode 1: 직선 경로로 움직임 (정밀)
            mc.send_coords(target, 40, 0)
            
        except ValueError:
            print("숫자만 입력 가능합니다.")

    # --- 그리퍼 입력 모드 ---
    elif key == ord('g'):
        try:
            val = int(input("그리퍼 값 (0-100): "))
            if 0 <= val <= 100:
                mc.set_gripper_value(val, 50)
            else:
                print("0~100 사이를 입력하세요.")
        except ValueError:
            print("숫자만 입력 가능합니다.")

    elif key == ord('q'):
        break

cv2.destroyAllWindows()
mc.send_angles(home_angles, 40)