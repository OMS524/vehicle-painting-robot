# 20260629_v1 백업

## 백업 목적

이 백업은 이후 `surface_marching`, guide curve 실험을 하기 전의 **축 기반 도장 경로 생성 알고리즘 원본**을 보존한 버전이다.

## 포함 파일

- `painting_trajectory_planner_1_surface_spline_extraction.py`
  - 축 기반 surface slicing.
  - work frame의 `slice_axis`, `row_axis`를 그대로 사용한다.
  - `slicing_method`, `surface_marching`, guide curve, local moving slicing plane 개념은 없다.
- `painting_trajectory_planner_2_correction.py`
  - 기존 2D 보정 알고리즘.
  - 각 slice마다 `connected component 추출 -> endpoint 검출 -> convex hull -> selected upper hull -> regularized corrected row` 순서로 처리한다.
- `painting_trajectory_planner_3_offset.py`
  - 기존 offset 생성 알고리즘.
  - offset 방향은 corrected row의 work frame `row-Z` 평면 기하 구조로 계산한다.
- `painting_trajectory_planner_4_spline.py`
  - B-spline 생성 및 trajectory sampling.
- `painting_trajectory_planner_5_structuring.py`
  - 최종 `PaintTrajectory`, `TrajectoryRow`, `TrajectoryPoint` 구조화.
- `painting_trajectory_planner.py`
  - 1~5단계 파이프라인 wrapper.
- `painting_trajectory.yaml`
  - 축 기반 slicing 시절 설정 파일.

## 핵심 알고리즘 상태

```text
축 기반 slicing
-> 2D convex-hull correction
-> row 형상 기반 offset
-> B-spline trajectory 생성
-> 구조화된 trajectory 출력
```

범퍼, 도어, 후드처럼 고정 축 기준 slicing으로도 문제가 적은 부품의 안정 기준 버전으로 보면 된다.
