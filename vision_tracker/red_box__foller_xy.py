import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# --- 1. MyCobot 설정 ---
port = "COM6" 

# [!!] 방향 설정 [!!]
DIR_PAN = 1.0   # 좌우(J1)
DIR_TILT = -1.0 # 상하(J4)

try:
    mc = MyCobot(port, 115200)
    print(f"MyCobot connected successfully on port {port}")
    
    # [!!] 시작 시 초기 자세 설정 [!!]
    print("Initializing: Setting initial pose...")
    current_angles_init = mc.get_angles()
    if current_angles_init:
        # 5번만 -90도로 하고 나머지는 현재 상태 유지 (또는 원하는 값으로 주석 해제 후 설정)
        # current_angles_init[1] = -20
        # current_angles_init[2] = 10
        # current_angles_init[3] = -20
        current_angles_init[4] = -90
        mc.send_angles(current_angles_init, 50)
        time.sleep(2.0)
        print("Initialization complete.")
    else:
        print("Error: Could not get angles for initialization.")
        exit()

    # [!!] 중요: 루프 시작 전에 '고정할 각도'를 미리 저장합니다 [!!]
    # 이 값들은 루프 내에서 절대 변하지 않습니다.
    fixed_angles = mc.get_angles()
    if not fixed_angles:
        print("Error: Could not get fixed angles.")
        exit()
        
    LOCK_J2 = fixed_angles[1]
    LOCK_J3 = fixed_angles[2]
    LOCK_J5 = fixed_angles[4] # -90도
    LOCK_J6 = fixed_angles[5]
    
    print(f"Locked Angles -> J2:{LOCK_J2}, J3:{LOCK_J3}, J5:{LOCK_J5}, J6:{LOCK_J6}")

except Exception as e:
    print(f"Error connecting to MyCobot on port {port}: {e}")
    exit()

# --- 2. 비전 처리 함수 ---
def find_red_boxes(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_boxes = []
    largest_contour = None
    max_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:
            if area > max_area:
                max_area = area
                largest_contour = contour

    if largest_contour is not None:
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            red_boxes.append((center_x, center_y)) 
            
    return red_boxes, image

# --- 3. 실시간 웹캠 및 로봇 제어 로직 ---

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("오류: 웹캠을 열 수 없습니다.")
    mc.close()
    exit()

print("웹캠을 시작합니다. 'q' 키를 누르면 종료됩니다...")

ret, frame = cap.read()
if not ret:
    print("오류: 첫 프레임을 읽을 수 없습니다.")
    cap.release()
    mc.close()
    exit()

frame_h, frame_w, _ = frame.shape
screen_center_x = frame_w // 2
screen_center_y = frame_h // 2
print(f"Frame dimensions: {frame_w}x{frame_h}, Center: ({screen_center_x}, {screen_center_y})")

# --- 제어 설정 (Tuning) ---
ROBOT_SPEED = 80
DEAD_ZONE = 20           

# [!!] 비례 제어 설정 [!!]
K_P_PAN = 1   # 좌우(J1) 민감도
K_P_TILT = 1  # 상하(J4) 민감도

# [!!] 최대 속도 제한 (안전장치) [!!]
MAX_SPEED_DPS = 80 

# 명령 주기
ROBOT_COMMAND_INTERVAL = 0.1 
last_robot_command_time = time.time()

last_vision_time = time.time()
latest_center_x = None
latest_center_y = None

while True:
    # --- 1. 비전 루프 ---
    current_time = time.time()
    delta_time = current_time - last_vision_time
    last_vision_time = current_time

    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임 수신 실패")
        break
    
    frame = cv2.flip(frame, 0) # 상하 반전
        
    red_boxes, image_with_boxes = find_red_boxes(frame)
    
    # 십자선 그리기
    cv2.line(image_with_boxes, (screen_center_x, 0), (screen_center_x, frame_h), (255, 0, 0), 1)
    cv2.line(image_with_boxes, (0, screen_center_y), (frame_w, screen_center_y), (255, 0, 0), 1)

    if red_boxes:
        center_x, center_y = red_boxes[0]
        cv2.circle(image_with_boxes, (center_x, center_y), 5, (0, 255, 255), -1)
        latest_center_x = center_x
        latest_center_y = center_y
    else:
        latest_center_x = None
        latest_center_y = None
        print("No red object detected.")

    cv2.imshow('MyCobot Rate Tracker', image_with_boxes)

    # --- 2. 로봇 제어 루프 ---
    if (current_time - last_robot_command_time) > ROBOT_COMMAND_INTERVAL:
        last_robot_command_time = current_time

        if latest_center_x is not None and latest_center_y is not None:
            
            # (A) 로봇 각도 읽기
            real_angles = mc.get_angles()
            if not real_angles:
                print("Warning: Could not read real-time angles.")
                continue 
                
            curr_j1 = real_angles[0] # 1번 (Pan)
            curr_j4 = real_angles[3] # 4번 (Tilt)
            
            # (B) 오차 계산
            error_x = latest_center_x - screen_center_x
            error_y = screen_center_y - latest_center_y 

            # (C) 이동할 각도 계산 변수 초기화
            move_j1 = 0.0
            move_j4 = 0.0

            # --- 좌우 (J1) 계산 ---
            if abs(error_x) > DEAD_ZONE:
                speed_pan = error_x * K_P_PAN * DIR_PAN
                speed_pan = np.clip(speed_pan, -MAX_SPEED_DPS, MAX_SPEED_DPS)
                move_j1 = speed_pan * ROBOT_COMMAND_INTERVAL

            # --- 상하 (J4) 계산 ---
            if abs(error_y) > DEAD_ZONE:
                speed_tilt = error_y * K_P_TILT * DIR_TILT
                speed_tilt = np.clip(speed_tilt, -MAX_SPEED_DPS, MAX_SPEED_DPS)
                move_j4 = speed_tilt * ROBOT_COMMAND_INTERVAL

            # 움직임이 있을 때만 명령 전송
            if abs(move_j1) > 0.01 or abs(move_j4) > 0.01:
                
                target_j1 = curr_j1 + move_j1
                target_j4 = curr_j4 + move_j4

                # 안전 범위 제한
                target_j1 = np.clip(target_j1, -165, 165)
                target_j4 = np.clip(target_j4, -165, 165)

                print(f"Err X:{error_x} Y:{error_y} | Move J1:{move_j1:.2f}, J4:{move_j4:.2f}")

                # [!!] 수정된 명령 전송 [!!]
                # real_angles(센서값) 대신 LOCK_JX(고정값)을 사용합니다.
                # [J1(목표), J2(고정), J3(고정), J4(목표), J5(고정), J6(고정)]
                mc.send_angles([target_j1, LOCK_J2, LOCK_J3, target_j4, LOCK_J5, LOCK_J6], ROBOT_SPEED)
            
            else:
                print("Object in Center (Deadzone). Holding.")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quitting...")
        cv2.destroyAllWindows()
        mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        break

cap.release()
# mc.release_all_servos()
print("Resources released. Exiting.")