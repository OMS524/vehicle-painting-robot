# 20260712_v3 백업

## 백업 목적

이 백업은 `surface_marching + guide curve` 구조에서 row 방향과 offset normal 계산을 scan direction 기준으로 바꾼 뒤의 상태를 보존한 버전이다.

## 포함 파일

- `painting_trajectory_planner_1_surface_spline_extraction.py`
  - `slicing_method: surface_marching` 사용.
  - 표면 중앙 strip에서 upper-hull 기반 guide curve를 만든다.
  - guide curve를 따라 local slicing plane을 만든다.
  - guide normal은 여전히 추정하고 부호 일관성 보정도 하지만, row/offset 방향의 주 기준으로 쓰지는 않는다.
  - 이 버전의 방향 계산 규칙:
    - `guide tangent = row가 쌓이는 진행 방향`.
    - `scan_dir` 기반 축 = 도장 줄(row)이 길게 지나가는 방향.
    - `row direction`은 scan direction에서 guide tangent 성분을 제거해 guide tangent와 수직이 되도록 보정한 방향이다.
    - `offset normal = guide tangent x row direction`.
  - scan direction이 guide tangent와 거의 평행해서 row 방향을 안정적으로 만들 수 없으면 surface normal 기반 row direction 방식으로 fallback한다.
- `painting_trajectory_planner_2_correction.py`
  - v1의 2D convex-hull correction 알고리즘 사용.
  - correction은 전역 축 기준이 아니라 각 local slicing plane 좌표계에서 수행한다.
- `painting_trajectory_planner_3_offset.py`
  - `slice_plane_other_axis_work`를 offset 방향으로 사용한다.
  - 이 버전에서 `slice_plane_other_axis_work`는 `guide tangent x scan-dir 기반 row direction`으로 계산된 방향이다.
  - offset 단계에서 work `+Z` 강제 flip은 적용하지 않는다.
- `painting_trajectory_planner_4_spline.py`
  - v1 백업과 동일한 알고리즘.
- `painting_trajectory_planner_5_structuring.py`
  - v1 백업과 동일한 알고리즘.
- `painting_trajectory_planner.py`
  - 변경된 설정 구조를 사용하는 파이프라인 wrapper.
- `painting_trajectory.yaml`
  - `surface_extraction.slicing_method: surface_marching` 사용.
  - 주요 튜닝 파라미터는 v2와 동일.
- `ES300h_trunk/`
  - 이 백업 시점에서 생성한 트렁크 테스트 결과물.

## 핵심 알고리즘 상태

```text
중앙 strip upper-hull guide curve 생성
-> guide curve 기반 local moving slicing plane 생성
-> local plane 좌표계에서 2D convex-hull correction
-> scan direction 기반 offset normal
-> B-spline trajectory 생성
-> 구조화된 trajectory 출력
```

## v2와의 핵심 차이

```text
v2 offset direction = 추정된 surface normal
v3 offset direction = guide tangent x scan-dir 기반 row direction
```

트렁크처럼 곡률이 큰 부품에서 surface normal의 좌우 흔들림이 row/offset 방향에 주는 영향을 줄일 수 있는지 확인하기 위한 실험 버전이다.
