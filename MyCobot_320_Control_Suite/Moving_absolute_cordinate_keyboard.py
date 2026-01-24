import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# 1. 로봇 연결 설정
mc = MyCobot('COM10', 115200) # 로봇 포트 설정
step_size = 1.0  # 이동 간격

# 2. MyCobot 320 하드웨어 가동 범위 (Software Limits) 설정
LIMITS = {
    'X': [-100.0, 350.0],    # 앞뒤 범위
    'Y': [-350.0, 350.0],    # 좌우 범위
    'Z': [40.0, 420.0],      # 높이 (바닥 충돌 방지를 위해 최소값 40 설정)
    'ROT': [-180.0, 180.0]   # Rx, Ry, Rz 회전 범위
}

# 3. 이미지 기반 초기 좌표 설정
initial_coords = [244.70, -86.20, 306.30, -178.39, 1.39, -92.77]
current_coords = list(initial_coords)

def check_limit(val, axis_key):
    """ 좌표가 범위를 벗어나지 않도록 클램핑(Clamping) 처리 """
    limit_range = LIMITS.get(axis_key, LIMITS['ROT'])
    # 최소값보다 작아지면 최소값 유지, 최대값보다 커지면 최대값 유지
    return max(limit_range[0], min(val, limit_range[1]))

def draw_info(img, coords):
    """ 화면에 실시간 좌표, 가이드 및 리미트 경고 출력 """
    img.fill(0)
    cv2.putText(img, "MyCobot 320 Safety Control Mode", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
    axis_keys = ['X', 'Y', 'Z', 'ROT', 'ROT', 'ROT']
    
    for i, label in enumerate(labels):
        val = coords[i]
        limit = LIMITS[axis_keys[i]]
        
        # 한계(Limit) 도달 시 글자색을 빨간색으로 변경
        is_at_limit = (val <= limit[0] or val >= limit[1])
        color = (0, 0, 255) if is_at_limit else (0, 255, 0)
        
        text = f"{label}: {val:.2f}"
        cv2.putText(img, text, (10, 80 + (i * 40)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    instr = ["W/S: X", "A/D: Y", "R/F: Z", "U/J/I/K/O/L: Rot", "Z/X: Grip", "P: Print", "ESC: Quit"]
    for i, text in enumerate(instr):
        cv2.putText(img, text, (260, 80 + (i * 30)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

# --- 메인 실행부 ---
if __name__ == "__main__":
    # 1. 서보 전원 활성화 및 그리퍼 초기화
    mc.power_on()
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()
    time.sleep(1)

    # 2. 시작 시 지정된 초기 위치로 자동 이동
    print(f"\n[시스템 초기화] MyCobot 320 기본 위치 이동: {initial_coords}")
    mc.send_coords(initial_coords, 40, 0)
    time.sleep(3)

    cv2.namedWindow('MyCobot_320_Safety_Control')
    gripper_val = 40

    while True:
        display = np.zeros((350, 500, 3), dtype=np.uint8)
        draw_info(display, current_coords)
        cv2.imshow('MyCobot_320_Safety_Control', display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27: break # ESC 종료
            
        moved = False
        prev_coords = list(current_coords)

        # --- 키보드 조작 및 가동 범위 제한 적용 ---
        # X축 이동
        if key == ord('w'): current_coords[0] = check_limit(current_coords[0] + step_size, 'X')
        elif key == ord('s'): current_coords[0] = check_limit(current_coords[0] - step_size, 'X')
        # Y축 이동
        elif key == ord('a'): current_coords[1] = check_limit(current_coords[1] + step_size, 'Y')
        elif key == ord('d'): current_coords[1] = check_limit(current_coords[1] - step_size, 'Y')
        # Z축 이동
        elif key == ord('r'): current_coords[2] = check_limit(current_coords[2] + step_size, 'Z')
        elif key == ord('f'): current_coords[2] = check_limit(current_coords[2] - step_size, 'Z')
        
        # 회전(Orientation) 이동
        elif key == ord('u'): current_coords[3] = check_limit(current_coords[3] + step_size, 'ROT')
        elif key == ord('j'): current_coords[3] = check_limit(current_coords[3] - step_size, 'ROT')
        elif key == ord('i'): current_coords[4] = check_limit(current_coords[4] + step_size, 'ROT')
        elif key == ord('k'): current_coords[4] = check_limit(current_coords[4] - step_size, 'ROT')
        elif key == ord('o'): current_coords[5] = check_limit(current_coords[5] + step_size, 'ROT')
        elif key == ord('l'): current_coords[5] = check_limit(current_coords[5] - step_size, 'ROT')
        
        # 그리퍼 제어
        elif key == ord('z'): 
            gripper_val = max(0, gripper_val - 10)
            mc.set_gripper_value(gripper_val, 50)
        elif key == ord('x'): 
            gripper_val = min(100, gripper_val + 10)
            mc.set_gripper_value(gripper_val, 50)

        # 좌표 출력 (복사용)
        elif key == ord('p'):
            print(f"Teaching Target -> Coords: {[round(c, 2) for c in current_coords]}, Grip: {gripper_val}")

        # 좌표가 실제로 변경되었을 때만 로봇에게 명령 전송
        if prev_coords != current_coords:
            # 속도 60으로 전송
            mc.send_coords(current_coords, 60, 0)

    cv2.destroyAllWindows()