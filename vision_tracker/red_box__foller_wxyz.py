import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# --- 1. MyCobot 설정 ---
port = "COM6" 

# [!!] 방향 설정 [!!]
DIR_PAN = 1.0    # 좌우(J1)
DIR_TILT = -1.0  # 상하(J4)
DIR_REACH = -1.0 # 앞뒤(J2) 

try:
    mc = MyCobot(port, 115200)
    print(f"MyCobot connected successfully on port {port}")
    
    # [!!] 초기 자세 설정 [!!]
    print("Initializing: Setting initial pose...")
    current_angles_init = mc.get_angles()
    if current_angles_init:
        current_angles_init[1] = 0     # J2 (Shoulder) - 직각
        current_angles_init[2] = 0     # J3 (Elbow) - 직각 (기둥이 수직)
        current_angles_init[3] = 0     # J4 (Wrist) - 정면
        current_angles_init[4] = -90   # J5 (Rotation) - 카메라 수평
        mc.send_angles(current_angles_init, 50)
        time.sleep(2.0)
        print("Initialization complete.")
    else:
        print("Error: Could not get angles for initialization.")
        exit()

    # [!!] 고정할 조인트 저장 [!!]
    fixed_angles = mc.get_angles()
    if not fixed_angles:
        print("Error: Could not get fixed angles.")
        exit()
        
    LOCK_J5 = fixed_angles[4] 
    LOCK_J6 = fixed_angles[5]

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
            red_boxes.append((center_x, center_y, max_area)) 
            
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

# --- 제어 설정 (Tuning) ---
ROBOT_SPEED = 80
DEAD_ZONE = 30           
# [!!] 수정: Deadzone을 줄여서 더 민감하게 반응하도록 변경 [!!]
DEAD_ZONE_AREA = 500     # (기존 3000 -> 500) 목표값 5000 기준 10% 오차 허용

# [!!] 목표 거리 설정 [!!]
TARGET_AREA = 5000.0 

# [!!] 비례 제어 게인 [!!]
K_P_PAN = 1.0       # J1 (좌우)
K_P_TILT = 1.0      # J4 (상하)
# [!!] 수정: 게인을 높여서 더 확실하게 움직이도록 변경 [!!]
K_P_REACH = 0.0005  # J2 (앞뒤) - (기존 0.0001 -> 0.0005)

# [!!] 속도 제한 [!!]
MAX_SPEED_DPS = 80.0

ROBOT_COMMAND_INTERVAL = 0.1 
last_robot_command_time = time.time()

last_vision_time = time.time()
latest_data = None 

while True:
    # --- 1. 비전 루프 ---
    current_time = time.time()
    delta_time = current_time - last_vision_time
    last_vision_time = current_time

    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.flip(frame, 0) # 상하 반전
    red_boxes, image_with_boxes = find_red_boxes(frame)
    
    # 십자선
    cv2.line(image_with_boxes, (screen_center_x, 0), (screen_center_x, frame_h), (255, 0, 0), 1)
    cv2.line(image_with_boxes, (0, screen_center_y), (frame_w, screen_center_y), (255, 0, 0), 1)

    if red_boxes:
        center_x, center_y, area = red_boxes[0]
        cv2.circle(image_with_boxes, (center_x, center_y), 5, (0, 255, 255), -1)
        
        info_text = f"Area: {int(area)} / Target: {int(TARGET_AREA)}"
        cv2.putText(image_with_boxes, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        latest_data = (center_x, center_y, area)
    else:
        latest_data = None
        cv2.putText(image_with_boxes, "No Object", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow('MyCobot Rate Tracker', image_with_boxes)

    # --- 2. 로봇 제어 루프 ---
    if (current_time - last_robot_command_time) > ROBOT_COMMAND_INTERVAL:
        last_robot_command_time = current_time

        if latest_data is not None:
            cx, cy, c_area = latest_data
            
            # (A) 로봇 각도 읽기
            real_angles = mc.get_angles()
            if not real_angles:
                continue 
                
            curr_j1 = real_angles[0] # Pan
            curr_j2 = real_angles[1] # Reach
            curr_j3 = real_angles[2] # Parallel Link
            curr_j4 = real_angles[3] # Tilt
            
            # (B) 오차 계산
            error_x = cx - screen_center_x
            error_y = screen_center_y - cy 
            error_area = TARGET_AREA - c_area 

            move_j1 = 0.0
            move_j2 = 0.0
            move_j3 = 0.0
            move_j4 = 0.0

            # 좌우 (J1)
            if abs(error_x) > DEAD_ZONE:
                speed_pan = error_x * K_P_PAN * DIR_PAN
                speed_pan = np.clip(speed_pan, -MAX_SPEED_DPS, MAX_SPEED_DPS)
                move_j1 = speed_pan * ROBOT_COMMAND_INTERVAL

            # 상하 (J4)
            if abs(error_y) > DEAD_ZONE:
                speed_tilt = error_y * K_P_TILT * DIR_TILT
                speed_tilt = np.clip(speed_tilt, -MAX_SPEED_DPS, MAX_SPEED_DPS)
                move_j4 = speed_tilt * ROBOT_COMMAND_INTERVAL

            # 앞뒤 (J2 & J3)
            if abs(error_area) > DEAD_ZONE_AREA:
                speed_reach = error_area * K_P_REACH * DIR_REACH
                # 앞뒤 속도는 안전을 위해 최대 속도의 절반으로 제한
                speed_reach = np.clip(speed_reach, -MAX_SPEED_DPS/2, MAX_SPEED_DPS/2)
                
                move_j2 = speed_reach * ROBOT_COMMAND_INTERVAL
                move_j3 = -move_j2 # J3는 J2와 반대로

                # [디버깅] 왜 움직이거나 안 움직이는지 확인용 출력
                # print(f"Area Err: {error_area:.0f} -> Speed: {speed_reach:.2f} -> Move: {move_j2:.2f}")

            if abs(move_j1) > 0.01 or abs(move_j4) > 0.01 or abs(move_j2) > 0.01:
                target_j1 = np.clip(curr_j1 + move_j1, -165, 165)
                target_j2 = np.clip(curr_j2 + move_j2, -140, 140)
                target_j3 = np.clip(curr_j3 + move_j3, -150, 150) 
                target_j4 = np.clip(curr_j4 + move_j4, -165, 165)

                print(f"Area:{int(c_area)} | Mv-> J1:{move_j1:.1f}, J2:{move_j2:.1f}, J4:{move_j4:.1f}")
                mc.send_angles([target_j1, target_j2, target_j3, target_j4, LOCK_J5, LOCK_J6], ROBOT_SPEED)
            else:
                # 디버깅: Deadzone 안에 있는지 확인
                # if abs(error_area) > DEAD_ZONE_AREA:
                #     print("Calculated move too small (< 0.01)")
                # else:
                #     print("Area OK (Inside Deadzone)")
                print(f"Area:{int(c_area)} | OK (Deadzone)")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quitting...")
        cv2.destroyAllWindows()
        mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        break

cap.release()
print("Exiting.")