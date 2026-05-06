# 도장 경로 목표
입력 데이터 : 표면 포인트클라우드
출력 데이터 : 표면 도장 경로


## 목표 작업물
해당 알고리즘은 차량 전체가 아닌 차량 부품을 목표로 한다.


## 목표 경로 패턴
고려하고 있는 경로 패턴은 다음과 같다.
1. 래스터 패턴
2. 사람이 도장하는 것처럼 사선 패턴
경로 패턴은 데이터를 잡는 기준이라고 생각한다.
어떤 패턴이든 해당 데이터에서 안정적인 도장 경로를 만들 수 있어야 한다.
따라서 어떤 패턴이든 해당 데이터에서 안정적인 도장 경로를 만들 수 있는 알고리즘이 있다면 경로 패턴은 단순 사용자의 선택의 영역이다.
그렇기에 기본적인 지그재그 형태인 래스터 패턴을 목표로 한다.
(현재 알고리즘이 슬라이싱을 기반으로 하고 있기에 사선 패턴으로 한다 하여도 이에 따라 슬라이싱 방향도 바꿔주면 되기에 문제가 없을 것이라 판단한다.)


## 목표 표면
**엣지 제외 표면** (ex. 앞면, 뒷면)
엣지 같이 얇은 면적의 표면 말고 어느정도 면적이 있는 표면을 목표로 한다.
대부분의 차량 작업물의 옆면은 거의 엣지이기 때문에 엣지는 다른 표면과 다르게 얇고 래스터 패턴에 맞지 않다.
구현하려면 다음과 같은 방법들이 제시된다.
1. 래스터 패턴의 표면 도장 경로에서 각 경로마다 시작점 또는 종료점에서 엣지 고려한 추가 경로 생성
(이 방식은 전체적인 엣지를 도장을 못한다. 특히 윗 엣지, 아랫 엣지)
2. 표면 데이터에서 외곽 데이터를 추출하여 엣지 도장 경로 생성 기능 추가
3. 별도 옆면 도장 경로 알고리즘

하지만 표면 도장이 제일 기본이 되어야 하기에 앞면, 뒷면을 목표로 개발을 진행하고
엣지는 표면 도장 경로 생성 알고리즘이 된 이후의 추가적인 개발 요소로 선정한다.


## 목표 경로
다음을 고려한 경로를 목표로 한다.
1. 전체적인 표면을 유지하는 경로
2. 래스터 경로에서 파여있는 부분(ex. 자동차 문 손잡이)에 대해서는 무시한 경로
3. 래스터 경로에서 차량 작업물의 뚫려있는 부분(ex. 창틀)은 양옆 경로와 자연스러운 경로






---





# 경로 생성 알고리즘
## 1. 슬라이싱 기반 점군 추출
1. 표면 포인트 클라우드를 작업 좌표계 기준으로 변환한다.
2. 변환된 데이터를 일정 간격으로 슬라이싱한다.
3. 각 슬라이스에서 설정한 두께만큼 상·하 범위의 데이터를 추출하여 슬라이스 높이에 맞춰 데이터 점군 추출한다.


## 2. 각 점군에서 표면 라인 추출 및 보정
1. 데이터 점군에서 **endpoint** 추출
> 슬라이스 점군을 (u, w) 평면에 투영한 뒤, 너무 가까운 중복 점들은 정리한다.
> 각 점에 대해 최근접 이웃 k개를 연결하여 그래프를 구성하고, 이 그래프의 연결 성분을 각각 하나의 컴포넌트로 정의한다.
> 각 컴포넌트에서 u값이 가장 작은 점을 시작 seed로 설정한 뒤, 다익스트라 알고리즘을 이용해 가장 멀리 있는 점을 탐색한다.
> 다시 해당 점을 시작점으로 다익스트라 알고리즘을 수행하여 반대편 끝점을 구하고, 두 점 사이 경로를 해당 컴포넌트의 대표 polyline으로 추출한다.
> 너무 짧은 대표 polyline은 노이즈로 간주하여 제외한다.
> 이후 각 컴포넌트의 endpoint 중 (u, w)가 가장 작은 쪽을 기준점으로 삼아 컴포넌트들을 (u, w) 오름차순으로 정렬한다.
> 정렬된 컴포넌트 순서를 유지한 채, 이전 polyline의 끝점과 더 가까운 endpoint가 다음 polyline의 시작점이 되도록 방향을 뒤집어 전체 polyline을 구성한다.
> 최종적으로 연결된 polyline의 시작점과 끝점을 전체 endpoint로 사용한다.

2. 데이터 점군의 Convex Hull에서 endpoint 기준 **Upper Convex Hull** 추출
> 데이터 점군으로부터 Convex Hull을 계산한다.
> 추출한 endpoint가 Hull Point에 정확히 포함되지 않을 수 있으므로, 각 endpoint와 가장 가까운 Hull Point를 해당 endpoint에 대응하는 점으로 간주한다.
> 두 대응 Hull Point 사이의 Convex Hull 경로는 두 개 존재하므로, 이를 upper hull 후보 경로로 설정한다.
> 두 endpoint를 잇는 선분(chord)을 기준선으로 두고, 각 후보 경로의 점들이 해당 기준선보다 w 방향으로 얼마나 위에 위치하는지를 상대 편차로 계산한다.
> 기준선 대비 w 방향 상대 편차가 더 크고, 더 일관되게 양의 값을 가지는 경로를 최종 upper hull로 선택한다.
> 기준선 대비 w 방향 상대 편차가 더 크고, 더 일관되게 양의 값을 가지는 경로를 최종 upper hull로 선택한다.

3. 샘플링 포인트에서 표면에 해당하는 **anchor** 영역과 해당하지 않는 **free** 영역 구분
> 추출한 Upper Hull을 따라 일정 간격으로 샘플링 포인트를 생성한다.
> 샘플링 포인트 h_i에 대해, 인접 샘플 h_{i-1}, h_{i+1}로부터 로컬 접선 벡터를 계산한다.
> h_i를 중심으로, 접선 방향으로 일정 폭을 갖는 탐색 구간 안의 데이터 점군을 선택한다.
> 선택된 점들 중 h_i와 가장 가까운 점까지의 거리가 임계값 이하이면 해당 샘플을 표면(anchor)으로 간주한다. 그렇지 않으면 해당 샘플을 free 영역으로 간주한다.

4. 샘플 점열 전체에 대해 Penalty 기반 **Quadratic Optimization** 수행
> 샘플링된 upper hull 점열 전체를 최적화 변수로 두고, 여러 penalty 항의 합이 최소가 되도록 보정 곡선을 계산한다.
> 이를 통해 표면이 존재하는 구간은 원래 형상을 유지하고, 빈 구간은 기존 흐름을 크게 깨지 않으면서 부드럽게 연결되도록 한다.
> 
> [목적함수]
> 1. endpoint 유지
> (||q_0-h_0||² + ||q_{N-1}-h_{N-1}||²)
> 
> 2. anchor 유지
> Σ a_i ||q_i - h_i||²
> 
> 3. 1차 차분 유지
> Σ e_i ||(q_i-q_{i-1}) - (h_i-h_{i-1})||²
> 
> 4. 2차 차분 최소화
> Σ ||q_{i-1} - 2q_i + q_{i+1}||²
> 
> **최종 목적함수**
> minimize
>   λ_end (||q_0-h_0||² + ||q_{N-1}-h_{N-1}||²)
> + λ_data   Σ a_i ||q_i - h_i||²
> + λ_stretch Σ e_i ||(q_i-q_{i-1}) - (h_i-h_{i-1})||²
> + λ_bend  Σ ||q_{i-1} - 2q_i + q_{i+1}||²


## 3. 보정된 점열 오프셋
1. 최적화된 점열을 재샘플링
2. 접선 계산
3. 인접 접선 일관성 검사
4. row 전체 tangent 방향을 chord 방향과 검사
5. chord의 u 방향 부호를 이용해 row 전체 회전 방향 결정
> g_u > 0 이면 한쪽 회전 (오른쪽 회전)
> g_u < 0 이면 반대 회전 (왼쪽 회전)

6. 결정된 고정 방향 회전으로 모든 점의 법선 생성
7. 계산된 법선 방향으로 오프셋 수행


## 4. 오프셋 점열 기반 trajectory 생성
1. 각 offset row를 입력받는다.
2. anchor point가 2개 이상이고 3D 좌표 형식인지 검증한다.
3. offset normal의 반대 방향을 분사 방향으로 사용한다.
4. anchor point의 누적 거리 값을 parameter로 보간형 B-Spline을 생성한다.
5. dense sampling으로 실제 spline arc length table을 계산한다.
6. 시작점과 끝점 tangent 방향으로 start/end extension을 생성한다.
7. extension을 포함한 전체 row의 arc-length table을 생성한다.
8. 전체 길이, 목표 속도, 최대 가속도로 사다리꼴 속도 프로파일을 생성한다.
9. `control_dt`마다 `t`, `s`, `path_speed`, `path_acceleration`을 샘플링한다.
10. 각 `s`에서 position을 계산한다.
11. 각 `s`에서 분사 방향을 보간한다.
12. 각 `s`에서 tangent를 계산한다.
13. tangent와 분사 방향으로 TCP orientation quaternion을 계산한다.
14. 표면 hit 검사를 수행한다.
15. core 구간 여부와 surface hit 결과를 합쳐 최종 `paint` mask를 만든다.
16. `linear_velocity = tangent * path_speed`를 계산한다.
17. quaternion 차분과 `path_speed`를 이용해 `angular_velocity`를 계산한다.
18. row별 trajectory 배열을 결과로 저장한다.


## 5. trajectory 데이터 구조화
4단계에서 생성한 시간 기반 trajectory 배열을 dataclass와 CSV로 구조화한다.

### 데이터 구조
```Python
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
