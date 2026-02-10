import numpy as np

def generate_path(
    u_min, u_max, v_min, v_max,
    row_spacing, point_spacing, standoff,
    surface_query,   # (u,v)->(p_surf(3,), n(3,))
    start_left=True,
):
    path_rows = []
    left_to_right = start_left

    vs = np.arange(v_max, v_min - 1e-12, -row_spacing)

    for v in vs:
        if left_to_right:
            us = np.arange(u_min, u_max + 1e-12, point_spacing)
        else:
            us = np.arange(u_max, u_min - 1e-12, -point_spacing)

        row = []
        for u in us:
            p_surf, n = surface_query(u, v)
            p = p_surf + standoff * n
            row.append(p)
        path_rows.append(np.asarray(row))
        left_to_right = not left_to_right

    return path_rows  # row 리스트로 반환(점프선 안 생김)
