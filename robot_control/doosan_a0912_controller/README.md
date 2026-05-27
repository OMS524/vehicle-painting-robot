








# 라이브러리

## Eigen
Eigen 설치
```bash
sudo apt update
sudo apt install libeigen3-dev
```

## Pinocchio
Pinocchio 설치
```bash
sudo apt install -y lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update
sudo apt install -y robotpkg-py3*-pinocchio
```

/opt/openrobots 환경변수 설정
```bash
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
```

```bash
cd /home/oms/vehicle-painting-robot/robot_control/doosan_a0912_controller/test
CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH cmake -S . -B build
cmake --build build -j4
```



지금 내가 계획하는게
현재 경로는 원하는 속도 x 100Hz로 나온 간격만큼 샘플링 했고 일단 선속도, 각속도, 사다리꼴 프로파일 등 구해서 트레젝토리를 만들긴했어
이걸 이용해서 NLP를 통해 최적값을 뽑고 TCP가 일정속도로 이동하게끔 제어하는건데 현재 로봇 제어는 리얼타임 주기가 1000Hz로 하고 있어
근데 NLP 결과가 포지션이여서 포지션 컨트롤 할려했는데 API 메뉴얼에서 포지션 컨트롤 API가 안정화가 안돼서 밸로시티 컨트롤 API 사용하라고 권장하더라고 그래서 밸로시티 컨트롤을 할 예정이야


[Offline]

1. TCP trajectory 준비
   - 기존 spline path X(s)
   - 기존 profile s(t), sdot(t), sddot(t)
   - CSV의 X_i 사용
   - 각 X_i에 대응되는 s_i도 확보

2. NLP 최적화
   Optimization variable:
   q_0, q_1, ..., q_N

   목적함수:
   1. Joint jump 최소화
      Σ ||q_i+1 - q_i||²

   2. Joint acceleration / smoothness 최소화
      Σ ||q_i+1 - 2q_i + q_i-1||²

   3. Manipulability 최대화
      -Σ w(q_i)

   constraint:
   1. Trajectory tracking
      FK(q_i) = X_i

   2. Joint position limit
      q_min ≤ q_i ≤ q_max

   3. Joint velocity limit
      |q_i+1 - q_i| / 0.01 ≤ qdot_max

   4. Joint acceleration limit
      |q_i+1 - 2q_i + q_i-1| / 0.01² ≤ qddot_max

   5. Singularity 회피
      w(q_i) ≥ w_min
      또는 σ_min(J(q_i)) ≥ ε

3. NLP 결과 저장
   (s_0, q_0)
   (s_1, q_1)
   ...
   (s_N, q_N)

4. q(s) 구성
   q = q(s)
   dq/ds 계산 가능하게 구성


[1000 Hz command 생성]

1. t_k = k * 0.001

2. 기존 profile에서 직접 계산
   s_k = s(t_k)
   sdot_k = sdot(t_k)

3. joint reference 계산
   q_ref(t_k) = q(s_k)

4. joint velocity reference 계산
   qdot_ref(t_k) = dq/ds(s_k) * sdot_k


[Online RT 1000 Hz]

1. q_actual 읽기

2. qdot_cmd 계산
   qdot_cmd = qdot_ref + Kp(q_ref - q_actual)

3. limit clamp

4. speedj_rt(qdot_cmd)






Constraint:
  position tolerance
  orientation angle tolerance
  joint position limit
  joint velocity limit
  joint acceleration limit

Objective:
  position error
  orientation angle error
  velocity smoothness
  acceleration smoothness
  joint center
  singularity avoidance