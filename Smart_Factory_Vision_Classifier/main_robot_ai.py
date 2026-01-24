import cv2
import numpy as np
import time
import os
import sys
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot
from collections import Counter

# ==========================================
# [1] YOLO 딥러닝 모델 설정
# ==========================================
model_path = 'best.pt'
if not os.path.exists(model_path):
    print(f"[오류] 같은 폴더에 '{model_path}' 파일이 없습니다!")
    sys.exit()

print("[시스템] YOLOv8 모델 로딩 중...")
model = YOLO(model_path)
print("[시스템] 모델 로드 완료!")

# ==========================================
# [2] 스마트 예측 함수 (중앙 우선 탐색)
# ==========================================
def predict_one_frame(frame):
    """ 
    화면 중앙에 가장 가까운 물체를 찾아서 자르고 예측 
    반환값: (색상이름, 잘린이미지, 박스좌표(x,y,w,h))
    """
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    # 1. 전처리 (이진화)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 주변이 어두우면 threshold를 조금 낮춰야 할 수도 있음 (80 -> 60)
    _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV) 
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return "None", None, None

    best_c = None
    min_dist = float('inf')

    # 2. 후보군 필터링 (가장 중앙에 있는 것 찾기)
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        
        # (조건1) 너무 작거나(노이즈), 너무 큰(배경/로봇팔) 것은 무시
        if w < 30 or h < 30: continue   # 너무 작음
        if w > 400 or h > 400: continue # 너무 큼 (화면 꽉 채우는 것 방지)

        # (조건2) 정사각형 모양인지 체크 (선택사항, 필요시 주석 해제)
        # aspect_ratio = float(w)/h
        # if aspect_ratio < 0.5 or aspect_ratio > 1.5: continue

        # (조건3) 화면 중앙과의 거리 계산
        obj_center_x = x + w // 2
        obj_center_y = y + h // 2
        dist = ((obj_center_x - center_x)**2 + (obj_center_y - center_y)**2)**0.5
        
        # 가장 중앙에 가까운 놈을 선택
        if dist < min_dist:
            min_dist = dist
            best_c = c

    if best_c is None:
        return "None", None, None

    # 3. 최종 선택된 윤곽선 자르기
    x, y, w, h = cv2.boundingRect(best_c)
    
    # 패딩 적용
    padding = 20
    crop_x = max(0, x + padding)
    crop_y = max(0, y + padding)
    crop_w = w - (padding * 2)
    crop_h = h - (padding * 2)

    if crop_w <= 0 or crop_h <= 0:
        return "None", None, None

    cropped = frame[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]

    # YOLO 추론
    results = model(cropped, verbose=False) 
    top1_index = results[0].probs.top1
    color_name = results[0].names[top1_index]
    
    return color_name.capitalize(), cropped, (x, y, w, h)

# ==========================================
# [3] 로봇 및 상수 설정
# ==========================================
try:
    mc = MyCobot('COM10', 115200) # 로봇 포트 설정
    print("[시스템] 로봇 연결 성공 (COM10)")
except Exception as e:
    print(f"[오류] 로봇 연결 실패: {e}")
    sys.exit()

BLOCK_HEIGHT = 25.0 
stack_count = 0      

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

red_defective_coords = {
    "이동하기": [70.7, -249.2, 306.3, -177.39, 2.39, -92.77],
    "물건 놓기": [70.7, -249.2, 182.3, -177.39, 2.39, -92.77],
    "위로 살짝 올리기": [70.7, -249.2, 306.3, -177.39, 2.39, -92.77]
}

# ==========================================
# [4] 메인 인식 함수 (디버깅 화면 강화)
# ==========================================
def detect_color_voting(duration=4.0):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return None

    print(f">>> {duration}초간 탐색 시작... (화면의 초록색 박스를 확인하세요)")
    
    detected_list = [] 
    start_time = time.time()
    target_colors = ['Blue', 'Green', 'Purple', 'Red']

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. 예측 수행
        current_color, cropped_img, box = predict_one_frame(frame)
        
        # 2. 결과 저장 (None이나 Background가 아닐 때만)
        if current_color != "None" and current_color != "Background":
            detected_list.append(current_color)

        # ---------------------------------------------------------
        # [화면 그리기] 원본 화면에 초록색 박스를 그려서 확인
        # ---------------------------------------------------------
        display_img = frame.copy()
        
        # (A) 인식된 박스 그리기
        if box is not None:
            x, y, w, h = box
            # 초록색 박스 (이게 큐브여야 함!)
            cv2.rectangle(display_img, (x, y), (x+w, y+h), (0, 255, 0), 3)
            # 위에 인식된 색상 글씨 쓰기
            cv2.putText(display_img, f"TARGET: {current_color}", (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            # 인식 안 되면 빨간 글씨
            cv2.putText(display_img, "NO OBJECT", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # (B) 투표 현황 표시
        elapsed = time.time() - start_time
        remain = duration - elapsed
        
        total_votes = len(detected_list)
        if total_votes > 0:
            counts = Counter(detected_list)
            y_offset = 50
            for color in target_colors:
                percent = (counts[color] / total_votes) * 100
                text_color = (200, 200, 200)
                if counts[color] == max(counts.values()):
                    text_color = (0, 255, 0) # 1등 강조

                cv2.putText(display_img, f"{color}: {percent:.1f}%", (10, y_offset), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                y_offset += 25

        cv2.putText(display_img, f"Time: {remain:.1f}s", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # (C) 잘린 이미지도 작게 구석에 보여주기 (옵션)
        if cropped_img is not None:
            try:
                # 우측 하단에 잘린 이미지 오버레이
                h_crop, w_crop = cropped_img.shape[:2]
                display_img[0:h_crop, 0:w_crop] = cropped_img
                cv2.rectangle(display_img, (0,0), (w_crop, h_crop), (255,0,0), 2)
            except:
                pass

        cv2.imshow('Robot Eye (Debugging)', display_img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

        if elapsed >= duration:
            break
    
    cap.release()
    cv2.destroyAllWindows()

    if not detected_list:
        print(">>> 감지된 물체가 없습니다.")
        return None
    
    winner_color, count = Counter(detected_list).most_common(1)[0]
    total_votes = len(detected_list)
    confidence = (count / total_votes) * 100

    print(f"\n[투표 결과] 총 {total_votes} 프레임 중 {count}표 획득")
    print(f"최종 결정: {winner_color} (비중: {confidence:.1f}%)")
    
    return winner_color

def move_robot(name, coords, grip):
    print(f"\n[실행] {name} 이동")
    mc.send_coords(coords, 20, 0)
    time.sleep(3)
    mc.set_gripper_value(grip, 20)
    time.sleep(2)

def run_process():
    global stack_count
    move_robot(process_steps[0]["name"], process_steps[0]["coords"], process_steps[0]["grip"])
    
    color = detect_color_voting(duration=4.0)
    
    if not color: 
        print("인식 실패. 공정을 중단합니다.")
        return

    print(f">>> 최종 확정된 색상: {color}")
    is_red = (color == "Red")

    for i in range(1, len(process_steps)):
        step = process_steps[i]
        current_coords = list(step["coords"])
        current_name = step["name"]

        if is_red:
            if "이동하기" in current_name: current_coords = red_defective_coords["이동하기"]
            elif "물건 놓기" in current_name: current_coords = red_defective_coords["물건 놓기"]
            elif "위로 살짝 올리기" in current_name: current_coords = red_defective_coords["위로 살짝 올리기"]
        else:
            if i == 5: 
                current_coords[2] += (stack_count * BLOCK_HEIGHT)

        move_robot(current_name, current_coords, step["grip"])
    
    if not is_red:
        stack_count += 1
        print(f"정상품 적재 완료 (현재: {stack_count}개)")
    else:
        print("불량 위치 배출 완료")

# --- 메인 실행부 ---
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(3)

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
        print("\n[종료] 프로그램을 종료합니다. 로봇을 원점([0,0,0,0,0,0])으로 이동합니다...")
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        time.sleep(4)  # 이동할 충분한 시간을 제공
        print("프로그램을 종료합니다.")
        break