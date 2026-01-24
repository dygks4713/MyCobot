import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# --- 1. MyCobot 설정 ---
port = "COM6" 
# [!!] 로봇의 실제 관절 방향에 따라 이 부호를 바꿔야 할 수 있습니다.
# 1.0 = 오른쪽으로 + / -1.0 = 오른쪽으로 -
JOINT_DIRECTION = 1.0 

try:
    mc = MyCobot(port, 115200)
    print(f"MyCobot connected successfully on port {port}")
    initial_angles = mc.get_angles()
    if not initial_angles:
        print("Error: Could not get initial angles from MyCobot.")
        exit()
    print(f"Initial 6-Axis robot angles (J1-J6): {initial_angles}")

except Exception as e:
    print(f"Error connecting to MyCobot on port {port}: {e}")
    exit()

# --- 2. 비전 처리 함수 (기존과 동일) ---
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
        # 가장 큰 객체의 중심 좌표만 반환
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            red_boxes.append((center_x, center_y)) # 각도(angle) 정보 제외
            
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

# --- 제어 설정 ---
# DEGREES_PER_SECOND = 5.0 # [!!] 삭제: 고정 속도 대신 비례 속도를 사용할 것입니다.
ROBOT_SPEED = 80         # send_angle()의 속도 (빠르게 설정해야 부드러움)
DEAD_ZONE = 20           # 중앙에서 +/- 20 픽셀 이내는 '가운데'로 간주 (떨림 방지)

# [!!] --- 새롭게 수정된 제어 설정 (비례 + 속도 제한) --- [!!]
# 1. 비례 게인: 오차(픽셀)를 '초당 각도(속도)'로 변환하는 비율입니다.
#    (값이 크면, 조금만 벗어나도 더 빠르게 반응합니다)
K_P_PAN = 2
# 2. 최대 속도 제한: "너무 각도가 크면 안돼"
#    오차가 아무리 커도 로봇은 1초에 15도 이상 움직이지 않습니다.
MAX_PAN_SPEED_DPS = 100.0 # (초당 15도)
# --------------------------------------------------------

# [!!] --- 수정된 부분 (Non-Blocking Timer) --- [!!]
ROBOT_COMMAND_INTERVAL = 0.1 # 0.1초에 한 번씩만 로봇에게 명령을 보냄 (더 부드러운 반응을 위해 0.2 -> 0.1로 줄임)
last_robot_command_time = time.time()
# ---------------------------------------------

# 비전 루프의 델타 타임 계산을 위한 초기 시간
last_vision_time = time.time()
# 로봇 제어 로직이 사용할 마지막으로 감지된 물체 위치
latest_center_x = None

while True:
    # --- 1. 비전 루프 (매우 빠르게 항상 실행) ---
    current_time = time.time()
    # 델타 타임(delta_time) 계산 (비전용)
    delta_time = current_time - last_vision_time
    last_vision_time = current_time

    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임 수신 실패")
        break
    
    # [!!] 카메라 영상을 상하 반전 (좌우는 그대로) [!!]
    frame = cv2.flip(frame, 0)

    red_boxes, image_with_boxes = find_red_boxes(frame)
    
    cv2.line(image_with_boxes, (screen_center_x, 0), (screen_center_x, frame_h), (255, 0, 0), 1)
    cv2.line(image_with_boxes, (0, screen_center_y), (frame_w, screen_center_y), (255, 0, 0), 1)

    if red_boxes:
        center_x, center_y = red_boxes[0]
        cv2.circle(image_with_boxes, (center_x, center_y), 5, (0, 255, 255), -1)
        # 로봇 루프가 사용할 수 있도록 최신 X좌표 저장
        latest_center_x = center_x
    else:
        # 물체가 안보이면 latest_center_x를 초기화
        latest_center_x = None
        print("No red object detected.")

    # [!!] cv2.imshow()를 로봇 제어 로직과 분리하여 항상 실행 (부드러운 화면)
    cv2.imshow('MyCobot Rate Tracker', image_with_boxes)

    # --- 2. 로봇 제어 루프 (천천히, 정해진 간격으로 실행) ---
    
    # "마지막 명령 후 0.2초가 지났는가?"
    if (current_time - last_robot_command_time) > ROBOT_COMMAND_INTERVAL:
        # 0.2초가 지났으면, 타이머 리셋
        last_robot_command_time = current_time

        # 로봇에게 명령을 보낼 차례
        if latest_center_x is not None:
            # 2. 로봇의 '실제' J1 각도 읽어오기
            real_angles = mc.get_angles()
            if not real_angles:
                print("Warning: Could not read real-time angles.")
                continue 
                
            current_j1 = real_angles[0]
            target_j1 = current_j1 # 기본값: 현재 위치 유지

            # 3. [!!] --- 여기가 핵심 수정 로직 --- [!!]
            
            # (1) 오차 계산 (방향 포함)
            error_x = latest_center_x - screen_center_x

            # (2) 물체가 Dead Zone 밖에 있을 때만 움직임
            if abs(error_x) > DEAD_ZONE:
                
                # (A) 오차에 비례한 '목표 속도' (초당 각도) 계산
                #     (JOINT_DIRECTION은 부호 조절용)
                target_speed_dps = error_x * K_P_PAN * JOINT_DIRECTION
                
                # (B) "너무 각도가 크면 안돼" -> 최대 속도 제한 (예: -15 ~ +15도/초)
                limited_speed_dps = np.clip(target_speed_dps, -MAX_PAN_SPEED_DPS, MAX_PAN_SPEED_DPS)
                
                # (C) 이번 명령 간격(0.1초) 동안 움직일 '실제 각도' 계산
                degrees_to_move = limited_speed_dps * ROBOT_COMMAND_INTERVAL

                print(f"Error: {error_x}px, Speed: {limited_speed_dps:.2f} dps, Move: {degrees_to_move:.2f} deg")

                # (D) 새로운 목표 각도 설정
                target_j1 = current_j1 + degrees_to_move
            
            else:
                # (E) 물체가 중앙에 있음 (Dead Zone)
                print("Object in Center. Holding.")
                pass
            
            # 5. 관절 각도 범위 제한 (안전 장치)
            target_j1 = np.clip(target_j1, -165, 165) # J1(Base)
            
            # 6. '새로운 목표' 각도로 로봇에게 명령 전송
            mc.send_angle(1, target_j1, ROBOT_SPEED)
        
        # (else: latest_center_x가 None이면, 즉 물체가 안보이면 아무 명령도 안 보냄)

    # 7. 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        mc.send_angles([0, 0, 0, 0, 0, 0], 50) # '0도' 자세로 복귀
        print("Quitting...")
        break

# 8. 자원 해제
cap.release()
time.sleep(1.5)
# mc.release_all_servos() # 서보 전원 해제
print("Resources released. Exiting.")