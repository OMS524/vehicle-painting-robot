# 20260712_v2 백업

## 백업 목적

이 백업은 `surface_marching + guide curve` 구조를 적용한 뒤, offset normal 계산을 `guide tangent x scan direction` 방식으로 바꾸기 전 상태를 보존한 버전이다.

## 포함 파일

- `painting_trajectory_planner_1_surface_spline_extraction.py`
  - `slicing_method: surface_marching` 추가.
  - 표면 중앙 strip에서 guide curve를 만든다.
  - guide curve는 correction 단계의 upper hull 아이디어를 이용해서 생성한다.
  - guide curve를 따라 local slicing plane을 만든다.
  - guide normal은 이전 normal과의 방향을 비교해서 부호가 갑자기 뒤집히지 않도록 일관성 보정한다.
  - row 방향과 offset 방향은 여전히 추정된 surface normal 기반이다.
    - `guide tangent`를 local surface tangent plane에 투영한다.
    - `row direction = surface normal x guide tangent`.
    - `offset direction = surface normal`.
- `painting_trajectory_planner_2_correction.py`
  - v1 백업의 2D convex-hull correction 알고리즘을 사용한다.
  - v1과 다른 점은, correction을 전역 축 기준 평면이 아니라 각 local slicing plane 좌표계에서 수행한다는 점이다.
  - slice 점들을 local `(row_axis, other_axis)` 좌표로 변환한 뒤 2D correction을 수행하고, 결과를 다시 3D work 좌표로 되돌린다.
- `painting_trajectory_planner_3_offset.py`
  - v1 백업의 offset 파이프라인 구조를 사용한다.
  - offset 방향은 `slice_plane_other_axis_work`에서 가져온다.
  - 이 버전에서 `slice_plane_other_axis_work`는 local surface normal이다.
- `painting_trajectory_planner_4_spline.py`
  - v1 백업과 동일한 알고리즘.
- `painting_trajectory_planner_5_structuring.py`
  - v1 백업과 동일한 알고리즘.
- `painting_trajectory_planner.py`
  - 변경된 설정 구조를 사용하는 파이프라인 wrapper.
- `painting_trajectory.yaml`
  - `surface_extraction.slicing_method: surface_marching` 사용.
  - `geodesic_neighbor_count`, `geodesic_normal_neighbor_count`, `geodesic_band_half_width` 등 surface_marching 관련 파라미터 포함.
- `ES300h_trunk/`
  - 이 백업 시점에서 생성한 트렁크 테스트 결과물.

## 핵심 알고리즘 상태

```text
중앙 strip upper-hull guide curve 생성
-> guide curve 기반 local moving slicing plane 생성
-> local plane 좌표계에서 2D convex-hull correction
-> surface normal 기반 offset
-> B-spline trajectory 생성
-> 구조화된 trajectory 출력
```

`guide tangent x scan direction` 기반 offset normal 실험 전 상태를 확인할 때 사용하는 백업이다.
