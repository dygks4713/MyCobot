import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot # [!!] 6축 MyCobot(280/320)용 라이브러리입니다.

# --- 1. MyCobot 6축 로봇 설정 ---
port = "COM6" 

try:
    mc = MyCobot(port, 115200)
    print(f"MyCobot (6-Axis) connected successfully on port {port}")
    current_angles = mc.get_angles() # [!!] 6개 관절(J1~J6)의 각도를 가져옵니다.
    if not current_angles:
        print("Error: Could not get initial angles from MyCobot.")
        print("Please check connection and power.")
        exit()
        
    print(f"Initial 6-Axis robot angles (J1-J6): {current_angles}")
    # J1(Pan, 베이스 회전)과 J4(Tilt, 헤드 상하)만 제어할 것입니다.
    current_j1 = current_angles[0] # J1
    current_j3 = current_angles[2] # [!!] 수정: J3(팔꿈치) 각도 추가
    current_j4 = current_angles[3] # J4

except Exception as e:
    print(f"Error connecting to MyCobot on port {port}: {e}")
K_TILT = 0.05 # 상하(J3, J4) 움직임 게인 (조정이 필요할 수 있습니다)
ROBOT_SPEED = 70 # 로봇 관절 이동 속도 (1~100)


# --- 2. 비전 처리 함수 (빨간 물체 찾기) ---
def find_red_boxes(image):
    # 이미지를 HSV로 변환
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 빨간 색상 범위 지정
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
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        
        cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
        
        angle = rect[2]
        
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            red_boxes.append((center_x, center_y, angle))
            
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

K_PAN = 0.05
K_TILT = 0.05
ROBOT_SPEED = 70


while True:
    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임 수신 실패")
        break
    
    frame = cv2.flip(frame, 0)
        
    red_boxes, image_with_boxes = find_red_boxes(frame)

    cv2.circle(image_with_boxes, (screen_center_x, screen_center_y), 5, (255, 0, 0), -1)

    if red_boxes:
        center_x, center_y, angle = red_boxes[0]
        
        cv2.circle(image_with_boxes, (center_x, center_y), 5, (0, 255, 255), -1)

        # =============================================================
        # [!!] 6축 로봇 관절 제어 로직
        # =============================================================
        
        # 1. 화면 중앙과 물체 중앙의 '오차(Error)'를 계산합니다.
        error_x = center_x - screen_center_x
        # [!!] 수정: cv2.flip(0)으로 상하가 반전되었으므로, error_y의 계산 방향을 반대로 뒤집습니다.
        error_y = screen_center_y - center_y

        print(f"Object at ({center_x}, {center_y}) | Error(x,y): ({error_x}, {error_y})")

        # 2. 이 '오차'를 0으로 만들기 위해 로봇 관절 각도를 조절합니다.
        
        # --- Pan (좌우) 제어 (J1) ---
        # error_x가 + (물체가 오른쪽에 있음) -> J1 각도를 '늘려서' 로봇을 오른쪽으로 돌림
        current_j1 += error_x * K_PAN
        
        # --- Tilt (상하) 제어 (J3 + J4 보상) ---
        # error_y를 'delta_y' (각도 변화량)로 변환합니다.
        delta_y = error_y * K_TILT
        
        # [!!] 수정: J3와 J4가 반대로 움직여 카메라 방향을 보상합니다.
        # (가정: J3 감소 = 팔 하강, J4 증가 = 헤드 보상)
        # 이 부호(+, -)는 실제 로봇 동작을 보고 반대로 바꿔야 할 수 있습니다.
        current_j3 -= delta_y
        current_j4 += delta_y


        # 3. 관절 각도 범위 제한 (안전 장치)
        # [!!] 누락되었던 코드입니다. (START)
        current_j1 = np.clip(current_j1, -165, 165) # J1(Base)
        current_j3 = np.clip(current_j3, -165, 165) # [!!] 수정: J3 각도 제한 추가
        current_j4 = np.clip(current_j4, -180, 180) # J4(Tilt)
        
        # 4. MyCobot에 명령 전송
        # J1(id=1)과 J4(id=4) 관절만 움직입니다.
        mc.send_angle(1, current_j1, ROBOT_SPEED)
        mc.send_angle(3, current_j3, ROBOT_SPEED) # [!!] 수정: J3 관절 명령 추가
        mc.send_angle(4, current_j4, ROBOT_SPEED)
        # [!!] 누락되었던 코드입니다. (END)

    else:
        print("No red object detected.")
    
    cv2.imshow('MyCobot 6-Axis Red Tracker', image_with_boxes)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quitting...")
        break

# 8. 자원 해제
cap.release()
cv2.destroyAllWindows()
# 로봇을 0점(초기) 자세로 이동
mc.send_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(1.5)
# 로봇 서보 전원 해제
mc.release_all_servos()
print("Resources released. Exiting.")