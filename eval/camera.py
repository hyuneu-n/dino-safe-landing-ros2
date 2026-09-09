#!/usr/bin/env python3
"""
camera.py — 하강캠 카메라 상수 + 픽셀<->world 변환 (eval/* 전용 공유 모듈)

landing_detector.py의 IMG_W/IMG_H/FX/FY/CX/CY/pixel_to_world() 를 그대로 복제한 것.
값은 worlds/generate_world.py의 DRONE_TEMPLATE에 박혀 있는 down_cam 센서 정의
(horizontal_fov=1.396, 640x480)에서 나온다 — 셋 중 하나라도 바꾸면 나머지 둘도 맞춰야 함.
landing_detector.py를 직접 import하지 않는 이유: rclpy/sensor_msgs 등 ROS 의존성을
끌고 오게 되어 eval 스크립트를 ROS 없이도 돌릴 수 없게 되기 때문 (베이스라인 비교는
저장된 프레임 + 알려진 드론 pose만 있으면 되고, 굳이 노드를 띄울 필요가 없음).
"""
import math

IMG_W, IMG_H = 640, 480
FX = (IMG_W / 2.0) / math.tan(1.396 / 2.0)   # ≈ 382, landing_detector.py와 동일
FY = FX
CX, CY = IMG_W / 2.0, IMG_H / 2.0


def pixel_to_world(u, v, h_alt, odom_xy):
    """landing_detector.LandingDetector.pixel_to_world() 와 동일한 공식."""
    off_a = (CX - u) / FX * h_alt
    off_b = (CY - v) / FY * h_alt
    ox, oy = odom_xy
    return ox + off_b, oy + off_a


def world_to_pixel(wx, wy, h_alt, odom_xy):
    """pixel_to_world()의 역변환 — GT 마스크가 world 좌표의 장애물을 픽셀로 투영할 때 사용."""
    ox, oy = odom_xy
    u = CX - (wy - oy) * FX / h_alt
    v = CY - (wx - ox) * FY / h_alt
    return u, v
