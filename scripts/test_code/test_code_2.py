import swift
import roboticstoolbox as rtb
import time

# 1. 로봇 모델 로드
urdf_path = "/home/oms/vehicle_painting_robot/models/urdf/a0912/a0912.white.urdf"
robot = rtb.ERobot.URDF(urdf_path)

# 2. Swift 환경 실행 및 로봇 추가
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

# 3. 루프 실행
while True:
    env.step(0.01)
    time.sleep(0.01)
