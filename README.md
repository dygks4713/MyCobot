# 🤖 MyCobot Vision & Control Suite

![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=flat-square&logo=python)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat-square&logo=opencv)
![PyTorch](https://img.shields.io/badge/PyTorch-AI_Model-EE4C2C?style=flat-square&logo=pytorch)
![ElephantRobotics](https://img.shields.io/badge/Hardware-myCobot_320-000000?style=flat-square)

**MyCobot Vision & Control Suite**는 6축 협동 로봇인 myCobot 320을 활용하여 스마트 팩토리 환경에서의 객체 분류(Vision Classification) 및 실시간 위치 추적(Object Tracking)을 구현한 프로젝트입니다.

<br>

## 🎥 video
### 로봇 작동(객체 분류)
> ![mycobot_vision_classifier1 (1)](https://github.com/user-attachments/assets/bdbd616d-34f1-4a40-a0e6-d1a1ea2b4587)
![1](https://github.com/user-attachments/assets/c2678571-7d82-424f-8747-bb1792575b01)
### 개발자 화면(AI)
>![KakaoTalk_20260409_142832422](https://github.com/user-attachments/assets/43401b41-46f0-48e8-8e06-bbced90dcebf)

<br>

## 🌟 Key Features

* **Smart Factory Vision Classifier (스마트 팩토리 비전 분류기)**
  * **AI 기반 분류 (`best.pt`):** 학습된 딥러닝 모델을 활용하여 로봇이 객체를 인식하고 자동으로 분류 및 이송합니다.
  * **HSV 색상 기반 제어:** 조명 변화에 강건한 HSV 색상 공간을 활용하여 특정 색상의 객체를 정밀하게 픽업합니다.
* **Real-time Vision Tracking (실시간 객체 추적)**
  * 카메라 영상을 기반으로 특정 객체(Red Box)를 인식하고 로봇 팔이 이를 실시간으로 따라갑니다.
  * 추적 난이도에 따라 단일 축(X), 평면(XY), 공간(WXYZ) 등 다양한 차원의 트래킹 알고리즘을 독립적으로 구현했습니다.
* **Kinematics & Control (기구학 및 제어)**
  * 절대 좌표(Absolute Coordinates) 기반의 로봇 암 이동 로직 구현.
  * 디버깅 및 수동 조작을 위한 키보드(Keyboard) 제어 인터페이스 지원.

<br>

## 📂 Repository Structure

```text
dygks4713-MyCobot/
├── MyCobot_320_Control_Suite/        # myCobot 320 제어 기본 모듈
│   ├── Absolute_coordinates.py       # 절대 좌표계 기반 이동 스크립트
│   └── Moving_absolute_cordinate_keyboard.py # 키보드 입력 기반 정밀 좌표 제어
│
├── Smart_Factory_Vision_Classifier/  # 비전 기반 객체 분류 및 조작 모듈
│   ├── best.pt                       # 학습 완료된 AI 비전 모델 가중치 파일
│   ├── main_robot_HSV.py             # HSV 색상 추적 기반 메인 로봇 제어
│   └── main_robot_ai.py              # AI 모델(best.pt) 연동 메인 로봇 제어
│
├── vision_tracker/                   # 카메라 기반 실시간 객체 추적 모듈 (Red Box)
│   ├── red_box__foller_x.py          # 1D (X축) 추적 알고리즘
│   ├── red_box__foller_xy.py         # 2D (X, Y축) 평면 추적 알고리즘
│   └── red_box__foller_wxyz.py       # 다차원(W, X, Y, Z) 복합 추적 알고리즘
│
├── test/                             # 알고리즘 검증 및 그리퍼(Gripper) 단위 테스트
│   ├── 1.py ~ 7.py, 4.pt
│   └── gripper.py
│
└── README.md
