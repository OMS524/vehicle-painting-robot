# ====================================================================================================
import numpy as np
import roboticstoolbox as rtb
import swift
# ====================================================================================================
from spatialmath import SE3
from spatialmath.base import q2r
import spatialmath.base as smb
from spatialgeometry import Box, Cylinder
# ====================================================================================================
from model_generation import create_cylinder, make_cylinder_half_query
from path_generation import generate_path
# ====================================================================================================







# =========================
# 너가 고정하라고 한 부분 (그대로)
# =========================
# urdf_path = "/home/oms/vehicle_painting_robot/models/urdf/a0912/a0912.white.urdf"
# robot = rtb.ERobot.URDF(urdf_path)

env = swift.Swift()
env.launch(realtime=True)
# env.add(robot)


# =========================
# 1) 모델 생성
# =========================
cyl_radius = 0.25
cyl_length = 0.50  # z축 방향 길이로 취급(대부분의 구현이 z축 길이)
cyl_pose = SE3(0.5, 0.0, 0.25)  # 중심 위치

cyl, cyl_meta = create_cylinder(env, cyl_radius, cyl_length, cyl_pose)

# =========================
# 2) "표면 데이터" 만들기: 샘플링해서 point cloud로
# =========================
cx, cy, cz = cyl_meta["cx"], cyl_meta["cy"], cyl_meta["cz"]
z_top, z_bot = cyl_meta["z_top"], cyl_meta["z_bot"]

theta_min, theta_max = -np.pi/2, np.pi/2
u_min, u_max = 0.0, cyl_radius*(theta_max-theta_min)
v_min, v_max = z_bot, z_top

surface_query = make_cylinder_half_query((cx,cy,cz), cyl_radius, theta_min, theta_max)


# =========================
# 3) 경로 생성 (파라미터는 전부 변수)
# =========================
row_spacing = 0.05     # "일정 간격 아래로" (행 간격)
point_spacing = 0.02   # 한 줄에서 점 간격
standoff = 0.20        # 표면으로부터의 이격 거리

rows = generate_path(
    u_min, u_max, v_min, v_max,
    row_spacing=0.05,
    point_spacing=0.02,
    standoff=0.10,
    surface_query=surface_query,
    start_left=True
)


# =========================
# 4) Swift 시각화: 포인트를 구(Sphere)로 띄우기
# =========================
# 너무 많이 찍으면 느릴 수 있어서, 필요하면 stride로 다운샘플
def add_polyline_as_cylinders(env, pts, radius=0.002, stride=1, color=(1.0, 0.0, 0.0)):
    # ✅ pts가 "세그먼트 리스트"면 각 세그먼트를 재귀적으로 그림
    if isinstance(pts, (list, tuple)):
        for seg in pts:
            add_polyline_as_cylinders(env, seg, radius=radius, stride=stride, color=color)
        return

    pts = np.asarray(pts, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return

    P = pts[::stride]
    z_axis = np.array([0.0, 0.0, 1.0])

    for a, b in zip(P[:-1], P[1:]):
        v = b - a
        L = float(np.linalg.norm(v))
        if L < 1e-9:
            continue

        d = v / L
        axis = np.cross(z_axis, d)
        s = np.linalg.norm(axis)
        c = float(np.dot(z_axis, d))

        if s < 1e-9:
            R = np.eye(3) if c > 0 else smb.rotx(np.pi)
        else:
            axis /= s
            angle = float(np.arctan2(s, c))
            R = smb.angvec2r(angle, axis)

        mid = (a + b) / 2.0
        T = SE3(mid) * SE3.Rt(R, [0, 0, 0])

        env.add(Cylinder(radius=radius, length=L, pose=T, color=color))

for row in rows:
    add_polyline_as_cylinders(env, row, radius=0.002, color=(1,0,0))


# =========================
# 루프
# =========================
# 카메라/업데이트 루프는 네 환경에 맞게
while True:
    env.step(0.02)

