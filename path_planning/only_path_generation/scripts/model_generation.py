import numpy as np
from spatialmath import SE3
from spatialgeometry import Cylinder

def create_cylinder(env, radius: float, length: float, pose: SE3, *, color=None):
    """
    env: swift.Swift()
    radius: 실린더 반지름
    length: 실린더 길이
    pose: SE3 (중심 pose)

    return: (cyl, meta)
      - cyl: spatialgeometry.Cylinder 객체
      - meta: 중심/상하단 z 등 편의 정보 dict
    """
    if color is None:
        cyl = Cylinder(radius=radius, length=length, pose=pose)
    else:
        cyl = Cylinder(radius=radius, length=length, pose=pose, color=color)

    env.add(cyl)

    cx, cy, cz = pose.t
    z_top = cz + length / 2.0
    z_bot = cz - length / 2.0

    meta = {
        "cx": cx, "cy": cy, "cz": cz,
        "z_top": z_top,
        "z_bot": z_bot,
        "radius": radius,
        "length": length,
        "pose": pose,
    }
    return cyl, meta

def make_cylinder_half_query(center_xyz, radius, theta_min, theta_max):
    cx, cy, _ = center_xyz
    # u = 호길이(0..R*(theta_max-theta_min)), v = z
    def query(u, v):
        theta = theta_min + u / radius
        theta = np.clip(theta, theta_min, theta_max)
        x = cx + radius*np.cos(theta)
        y = cy + radius*np.sin(theta)
        z = v
        p_surf = np.array([x,y,z], float)
        n = np.array([np.cos(theta), np.sin(theta), 0.0], float)
        return p_surf, n
    return query







