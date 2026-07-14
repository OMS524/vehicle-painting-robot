# Painting Trajectory Planner

표면 포인트 클라우드를 입력받아 차량 부품 표면을 따라가는 도장 trajectory를 생성하는 알고리즘이다.

현재 구현은 **axis slicing**과 **surface marching slicing**을 선택할 수 있다. 생성된 surface row를 보정한 뒤 TCP offset, B-Spline, 사다리꼴 속도 프로파일을 거쳐 최종 trajectory CSV를 생성한다.

## 목표

- 입력 데이터: 차량 부품 표면 포인트 클라우드 (`csv`, `ply`)
- 출력 데이터: row별 도장 trajectory CSV
- 대상 부품: 차량 전체가 아니라 hood, door, bumper, trunk 같은 개별 부품
- 대상 표면: 엣지/얇은 측면보다 넓은 주 표면
- 기본 경로 패턴: raster / zigzag 기반 row trajectory

현재 알고리즘은 표면 도장 경로를 우선 목표로 한다. 엣지 전용 도장 경로는 별도 알고리즘 또는 후처리로 추가하는 것이 맞다.

## 전체 파이프라인

```text
surface point cloud
→ 1. surface spline extraction
→ 2. correction
→ 3. offset
→ 4. spline trajectory generation
→ 5. structuring / csv export
```

주요 구현 파일은 `painting_trajectory_planner/scripts` 아래에 단계별로 나뉘어 있다.

```text
painting_trajectory_planner.py
painting_trajectory_planner_1_surface_spline_extraction.py
painting_trajectory_planner_2_correction.py
painting_trajectory_planner_3_offset.py
painting_trajectory_planner_4_spline.py
painting_trajectory_planner_5_structuring.py
```

## 1. Surface Spline Extraction

입력 point cloud를 작업 좌표계로 변환한 뒤 일정 간격으로 slice를 생성한다.

작업 좌표계는 다음 방향으로 구성된다.

- `paint_dir`: 분사 방향, gun에서 부품을 향하는 방향
- `scan_dir`: row가 진행하는 기준 방향
- `surface_dir`: 표면에서 gun 쪽으로 향하는 방향

기본 slicing 방식은 축 기반이다.

1. point cloud 평균점을 기준으로 작업 좌표계 생성
2. `spline_start_side`에 따라 slice axis와 row axis 결정
3. `spline_spacing` 간격으로 slice 위치 생성
4. 각 slice 위치에서 `spline_half_width` 범위 안의 점군 추출
5. 추출한 점들의 slice axis 값을 slice 중심값으로 고정

출력은 `slice_profiles`이며 각 profile은 다음 데이터를 가진다.

```text
slice_position
slice_points_work
slice_points_world
```

`surface_extraction.slicing_method: surface_marching`로 설정하면 표면을 따라 전진하는 marching slicing을 사용한다. 기존 `geodesic` 값도 호환된다.

1. point cloud를 작업 좌표계로 변환
2. PCA 기반 local surface normal 추정
3. point cloud를 `(row axis, slice axis)` occupancy cell로 나누고 `guide_extension_min_bin_points`개 이상인 cell만 유효 처리
4. 각 row strip의 cell이 양옆 strip 중 하나 이상에서도 반복되는지 검사해 고립 노이즈 제거
5. 중간의 빈 cell은 구멍으로 허용하고, 안정적으로 지지되는 최하단·최상단 범위가 가장 긴 row strip 선택
6. 선택한 strip과 양옆 strip의 모든 점에서 upper hull을 추출해 최종 guide 생성
7. 최종 guide point의 tangent를 법선으로 사용하는 local slicing plane 생성
8. 각 local slicing plane의 `± geodesic_band_half_width` slab 안에 있는 모든 표면 점 추출
9. 점이 `geodesic_min_band_points`보다 적을 때만 slab 폭을 단계적으로 확장
10. 추출한 전체 점군을 local plane 좌표계의 correction 단계로 전달

슬라이싱 평면 안의 점군이 중앙, 좌측, 우측처럼 여러 연결 성분으로 나뉘어 있어도 모두 `slice_points_work/world`에 보존한다. 가이드가 포함된 연결 성분만 선택하지 않는다.

surface_marching mode의 `slice_position`은 row index이며, 실제 row 간격은 이전 row의 local surface frame을 따라 marching한 `spline_spacing`이다.

## 2. Correction

각 slice에서 추출한 점군을 하나의 표면 row로 정리하고 보정한다.

axis mode의 correction은 축 기반 slice를 전제로 한다. 각 slice point를 작업 좌표계의 2D 평면으로 투영한다.

```text
u = row_axis 좌표
w = z 좌표
points_uw = (u, w)
```

### 2.1 Endpoint / Component Path 추출

1. slice 점군을 `(u, w)` 평면에 투영
2. 가까운 중복점을 `quantize_step` 기준으로 정리
3. 각 점을 최근접 이웃 `k`개와 연결해 kNN graph 구성
4. graph connected component 분리
5. 각 component에서 Dijkstra 기반 longest path 추출
6. 너무 짧은 component는 노이즈로 제거
7. component를 left/top 기준으로 정렬
8. 정렬된 component를 순서대로 병합
9. 병합된 path의 시작점과 끝점을 전체 endpoint로 사용

현재 component 병합은 다음 backtrack 검사를 포함한다.

```text
다음 component 시작점이 현재 끝점보다 왼쪽으로
component_merge_backtrack_tolerance 이상 돌아가면 해당 component를 건너뜀
```

### 2.2 Convex Hull Chain 선택

1. slice 점군의 2D convex hull 계산
2. endpoint와 가장 가까운 hull point를 각각 찾음
3. 두 hull point 사이의 후보 chain 두 개 생성
4. endpoint chord 기준으로 더 위쪽에 놓이는 chain을 선택
5. 선택된 chain을 correction 기준 upper hull로 사용

### 2.3 Anchor / Free 구간 판단

선택된 hull chain을 일정 간격으로 샘플링하고, 각 샘플이 실제 표면 점군의 지지를 받는지 판단한다.

- anchor: 주변 표면 점과 충분히 가까운 샘플
- free: 주변 표면 점이 없어 부드럽게 연결해야 하는 샘플

### 2.4 Quadratic Optimization

upper hull 샘플 전체를 최적화 변수로 두고 보정 곡선을 계산한다.

목적함수는 다음 항으로 구성된다.

```text
endpoint 유지
anchor 유지
1차 차분 유지
2차 차분 최소화
```

최종 목적함수:

```text
minimize
  lambda_end     * endpoint_error
+ lambda_data    * anchor_error
+ lambda_stretch * first_difference_error
+ lambda_bend    * second_difference_error
```

출력은 다음 debug 데이터를 포함한다.

```text
corrected_rows
corrected_row_profiles
correction_observed_path_rows_world
correction_connected_component_rows_world
correction_convex_hull_rows_world
correction_selected_hull_rows_world
correction_supported_sample_rows_world
correction_endpoint_points_world
```

surface_marching mode의 correction은 2D convex hull correction을 사용하지 않는다. 각 marched band 안에서 3D surface graph를 다시 구성하고, 가장 긴 대표 component path를 선택한 뒤 arc-length 기준으로 재샘플링하여 `corrected_rows`를 만든다.

surface_marching correction 출력은 다음을 포함한다.

```text
corrected_rows
corrected_row_profiles
correction_observed_path_rows_world
correction_connected_component_rows_world
correction_endpoint_points_world
```

## 3. Offset

보정된 surface row를 TCP trajectory 기준 위치로 offset한다.

1. corrected row를 `offset_point_spacing` 간격으로 재샘플링
2. row tangent 계산
3. tangent 방향 일관성 정리
4. row chord 방향을 기준으로 normal 방향 결정
5. 계산된 normal 방향으로 `offset_distance`만큼 이동

출력은 다음과 같다.

```text
offset_rows
offset_points_world
offset_row_normals_world
```

axis mode에서는 offset normal을 각 row의 2D 곡선 형태를 기준으로 계산한다. surface_marching mode에서는 surface extraction에서 추정한 surface normal을 우선 사용하고, normal 데이터가 없으면 기존 2D tangent 기반 normal로 fallback한다.

## 4. Spline Trajectory Generation

offset row를 입력받아 실제 제어 주기에 맞는 시간 기반 trajectory를 생성한다.

각 row 처리 순서:

1. offset row anchor point 검증
2. offset normal의 반대 방향을 분사 방향으로 사용
3. anchor point 누적 거리 기준 interpolating B-Spline 생성
4. dense sampling으로 spline arc-length table 생성
5. 시작점/끝점 tangent 방향으로 extension 생성
6. extension 포함 전체 row의 arc-length table 생성
7. 전체 길이, 목표 속도, 최대 가속도로 사다리꼴 속도 프로파일 생성
8. `control_dt`마다 `t`, `s`, `path_speed`, `path_acceleration` 샘플링
9. 각 `s`에서 position 계산
10. 각 `s`에서 분사 방향 보간
11. 각 `s`에서 tangent 계산
12. tangent와 분사 방향으로 TCP orientation quaternion 계산
13. 표면 hit 검사
14. core 구간과 surface hit 결과를 합쳐 paint mask 생성
15. `linear_velocity = tangent * path_speed` 계산
16. quaternion 차분과 path speed로 angular velocity 계산
17. row별 trajectory 배열 저장

`raster_zigzag`가 켜져 있으면 row 진행 방향을 한 줄씩 번갈아 뒤집는다. 이때 position, tangent, velocity, acceleration, paint mask도 함께 뒤집힌다.

## 5. Structuring / CSV Export

4단계에서 생성한 row별 trajectory 배열을 dataclass로 구조화하고 CSV로 저장한다.

```python
@dataclass(slots=True)
class TrajectoryPoint:
    t: float
    s: float
    position: Vec3
    orientation: Quat
    linear_velocity: Vec3
    angular_velocity: Vec3
    path_speed: float
    path_acceleration: float
    paint: bool


@dataclass(slots=True)
class TrajectoryRow:
    row_index: int
    points: list[TrajectoryPoint] = field(default_factory=list)


@dataclass(slots=True)
class PaintTrajectory:
    frame_id: str
    control_dt: float
    desired_speed: float
    max_acceleration: float
    rows: list[TrajectoryRow] = field(default_factory=list)
```

CSV에는 다음 정보가 저장된다.

```text
row_index
point_index
frame_id
t
s
position
orientation
linear_velocity
angular_velocity
path_speed
path_acceleration
paint
```

## 주요 파라미터

기본 파라미터는 `painting_trajectory_planner/config/painting_trajectory.yaml`에서 관리한다.

```text
surface_extraction.spline_spacing
surface_extraction.spline_half_width
surface_extraction.slicing_method
surface_extraction.geodesic_neighbor_count
surface_extraction.geodesic_normal_neighbor_count
surface_extraction.geodesic_edge_max_factor
surface_extraction.geodesic_seed_width
surface_extraction.geodesic_seed_bin_spacing
surface_extraction.geodesic_band_half_width
surface_extraction.guide_extension_min_bin_points
surface_extraction.geodesic_min_band_points
correction.endpoint.graph_neighbor_count
correction.endpoint.component_merge_backtrack_tolerance
correction.support.support_point_spacing
correction.optimization.lambda_*
offset.offset_point_spacing
offset.offset_distance
spline.spline_point_spacing
spline.endpoint_extension_length
spline.raster_zigzag
trajectory.desired_speed
trajectory.control_dt
trajectory.max_acceleration
```

## 현재 한계

axis slicing은 door, hood, bumper처럼 곡률이 비교적 완만하거나 기준축 slicing이 표면 흐름과 잘 맞는 부품에서 안정적이다. 하지만 trunk처럼 곡률이 크고 표면 흐름이 축과 크게 어긋나는 부품에서는 row가 표면을 자연스럽게 따라가지 못할 수 있다.

surface_marching slicing은 이 문제를 줄이기 위해 추가된 표면 추종 방식이다. 현재 surface_marching mode는 v1 구현이며 다음 특성이 있다.

```text
현재 row의 tangent와 local surface normal로 다음 row 방향 계산
local spacing direction으로 `spline_spacing`만큼 전진
marched band 내부에서 가장 긴 대표 path 선택
band path는 arc-length resampling만 수행
기존 B-Spline trajectory 생성 단계는 그대로 사용
```

현재 선택식 구조는 다음과 같다.

```yaml
surface_extraction:
  slicing_method: axis   # axis | surface_marching
```

이렇게 하면 기존 부품에서는 안정적인 축 기반 slicing을 유지하고, trunk 같은 부품에서만 surface_marching slicing을 비교 적용할 수 있다.
