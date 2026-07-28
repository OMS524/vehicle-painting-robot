# 20260722_v4 백업

## 백업 목적

이 백업은 v3의 `surface_marching` 구조를 정리하고, 슬라이싱 방식을 `axis`와
`surface_adaptive` 두 모드로 분리한 상태를 보존한 버전이다.

주요 목적은 다음과 같다.

- 범퍼처럼 중앙보다 다른 위치의 상하 범위가 더 긴 부품에서도 적절한 가이드라인 선택
- 각 슬라이싱 평면에 포함된 데이터를 끊어진 component 단위로 버리지 않고 모두 사용
- 두 슬라이싱 모드 모두에서 v1의 탄젠트 기반 offset 계산식 사용
- 분사 T/F를 각 row가 생성된 실제 슬라이싱 평면 기준으로 판정

## v3 대비 핵심 변경 사항

### 1. 슬라이싱 모드 정리

- 지원 모드를 `axis`와 `surface_adaptive` 두 가지로 제한했다.
- 기존 `surface_marching` 명칭은 제거했다.
- `axis`는 도장 방향과 스캔 방향으로 만든 작업 좌표계의 고정 축을 기준으로 슬라이싱한다.
- `surface_adaptive`는 점군 형상에서 선택한 Upper Hull 가이드라인을 따라 움직이는
  local slicing plane을 사용한다.

### 2. surface_adaptive 가이드라인 선택 방식 변경

v3에서는 부품 중앙의 제한된 strip에서 Upper Hull을 구했다. v4에서는
`(row axis, slice axis)` occupancy를 이용해 유효한 상하 범위가 가장 긴 strip을 먼저
선택하고, 해당 strip의 점들로 Upper Hull 가이드라인을 만든다.

- occupancy cell에 `guide_extension_min_bin_points`개 이상의 점이 있어야 유효한 데이터로 본다.
- 단일 위치의 노이즈를 줄이기 위해 현재 strip과 인접 strip에서 반복 지지되는 cell을 사용한다.
- 가운데에 구멍이나 빈 구간이 있는 부품을 고려해, 상단과 하단 사이의 내부 빈 bin은 허용한다.
- 유효한 상하 범위가 가장 긴 strip을 우선하고, 범위가 같으면 점 지지량과 안정적인 cell 수를
  이용해 선택한다.
- 선택된 strip과 인접 strip의 점을 이용해 최종 Upper Hull을 계산한다.

### 3. 각 슬라이싱 평면의 데이터 추출 방식 변경

v3에서는 local slicing slab 안의 점들을 연결 component로 나눈 뒤 특정 component만 선택했다.
이 방식은 같은 슬라이싱 평면에 데이터가 떨어져 존재하는 범퍼에서 일부 표면을 누락할 수 있었다.

v4에서는 local slicing plane의 slab 범위 안에 들어온 **모든 점**을 row 원본 데이터로 사용한다.
따라서 가운데가 비어 있거나 서로 떨어진 표면도 같은 슬라이싱 평면에 포함되면 함께 추출된다.

### 4. offset을 v1 방식으로 통일

v3의 고정 `slice_plane_other_axis_work` 방향 offset을 제거했다. v4에서는 `axis`와
`surface_adaptive` 모두 다음 순서로 동일하게 처리한다.

1. corrected row를 해당 row의 local slicing plane 2D 좌표로 투영한다.
2. v1과 같은 탄젠트 및 chord 부호 판정으로 90도 회전한 offset normal을 계산한다.
3. 계산한 normal을 작업 좌표계와 월드 좌표계로 다시 변환한다.

즉, offset 계산식은 v1 방식을 그대로 사용하고 계산 좌표계만 각 슬라이싱 평면에 맞춘 것이다.
추가적인 점별 `+work Z` 강제 반전은 적용하지 않는다.

### 5. 분사 T/F 판정을 실제 슬라이싱 평면 기준으로 변경

- offset row마다 원본 `slice_profile` 정보를 `offset_row_profiles`로 보존한다.
- 분사 판정 시 단순히 가장 가까운 전역 축 슬라이스를 찾지 않고, 해당 row가 생성된 원본
  슬라이싱 평면을 우선 연결한다.
- trajectory sample, 분사 방향, 원본 슬라이스 점들을 같은 local plane 좌표계로 투영한 뒤
  표면 교차 여부를 판정한다.
- 따라서 `axis`와 `surface_adaptive` 모두 각 슬라이싱 평면별로 분사 T/F가 계산된다.

## 설정 변경

- `surface_extraction.slicing_method`
  - `surface_marching`에서 `surface_adaptive`로 변경
- `surface_extraction.guide_extension_min_bin_points: 8`
  - 가이드 strip occupancy cell의 최소 점 개수로 추가
- `geodesic_neighbor_count`, `geodesic_edge_max_factor`
  - 이전 component 방식과의 설정 호환을 위해 남아 있지만 현재 all-points slab 추출에서는 미사용
- `offset.offset_distance`
  - `0.15 m`에서 `0.3 m`로 변경

## 변경되지 않은 부분

- `painting_trajectory_planner_2_correction.py`
  - v3와 동일하며, 각 local slicing plane 좌표계에서 v1의 2D convex-hull correction을 수행한다.
- `painting_trajectory_planner_5_structuring.py`
  - v3와 동일하다.

## 현재 처리 흐름

```text
axis 고정 슬라이싱 또는 longest-supported-strip Upper Hull 생성
-> 각 슬라이싱 평면 slab의 모든 점 추출
-> local plane 좌표계에서 2D convex-hull correction
-> 각 local plane에서 v1 탄젠트 기반 offset
-> B-spline trajectory 생성
-> 원본 local slice plane 기준 분사 T/F 판정
-> 구조화된 trajectory 출력
```

## 포함된 테스트 결과

| 부품 | 생성 row 수 |
|---|---:|
| ES300h_front_bumper | 14 |
| ES300h_hood | 33 |
| ES300h_left_fender | 18 |
| ES300h_trunk | 16 |
| NX350h_right_fender | 21 |
| Tesla_Model_Y_front_bumper | 13 |

각 결과 폴더에는 통합 `painting_trajectory.csv`, row별 `trajectory_NNN.csv`,
`trajectory_debug.html`이 포함되어 있다.

## 주의 사항

- v1 백업의 기본 offset 거리는 `0.15 m`였지만 이 백업 설정은 `0.3 m`이다. v1 결과와 좌표를
  직접 비교할 때는 offset 거리를 동일하게 맞춰야 한다.
- `surface_adaptive`의 내부 빈 bin 허용은 부품의 실제 홀을 보존하기 위한 것이며, 상하 경계는
  인접 strip의 반복 지지를 요구해 고립된 노이즈의 영향을 줄인다.
