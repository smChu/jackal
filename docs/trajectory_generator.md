# Task 3-1: 가상 보행자 궤적 생성기

Python 3 표준 라이브러리만 사용하는 궤적 생성기입니다. ROS 2, Gazebo, 추가 pip 패키지 설치 없이 실행할 수 있습니다. 저장소 루트에서 아래 명령을 실행하세요.

## 실행 예시

```bash
# 직선: (-2, -2)에서 (2, -2)까지 4초간 이동, 10 Hz
python3 scripts/generate_trajectory.py --scenario straight \
  --start-x -2 --start-y -2 --distance 4 --duration 4 \
  --output trajectories/straight.csv

# 좌회전: (-2, -2)에서 (0, 0)까지 반지름 2 m, 90도 원호
python3 scripts/generate_trajectory.py --scenario turn \
  --start-x -2 --start-y -2 --radius 2 --turn-deg 90 --duration 4 \
  --output trajectories/turn.csv

# 우회전: (-2, 2)에서 (0, 0)까지 반지름 2 m, -90도 원호
python3 scripts/generate_trajectory.py --scenario turn \
  --start-x -2 --start-y 2 --radius 2 --turn-deg -90 --duration 4 \
  --output trajectories/right_turn.csv

# 원형: (0, -2)에서 출발, 중심 (0, 0), 반지름 2 m, 반시계 방향 한 바퀴
python3 scripts/generate_trajectory.py --scenario circle \
  --start-x 0 --start-y -2 --radius 2 --duration 12 --rate 20 \
  --output trajectories/circle.csv
```

위 예시는 현재 보행자 이동 구역인 `x, y ∈ [-2, 2] m` 안에 위치합니다. 생성기는 구역 제한이나 장애물 회피를 적용하지 않으므로, 시작점과 경로 크기를 바꾸면 이동 구역을 직접 확인해야 합니다.

## CSV 규약

헤더는 항상 아래 순서입니다. 기본 실행(`python3 scripts/generate_trajectory.py`)의 처음 세 행은 다음과 같습니다.

```csv
person_id,timestamp,x,y,yaw
walking_person,0.0,0.0,0.0,0.0
walking_person,0.1,0.05,0.0,0.0
walking_person,0.2,0.1,0.0,0.0
```

| 필드 | 의미 |
| --- | --- |
| `person_id` | 보행자 식별 문자열. 기본값은 현재 SDF actor 이름인 `walking_person` |
| `timestamp` | 궤적 시작 기준 경과 시간(초). 기본 0부터 시작하며 `--start-time`으로 오프셋 지정 |
| `x`, `y` | Gazebo world XY 평면의 위치(m)로 사용할 좌표 |
| `yaw` | 이동 방향(rad), 범위 `[-π, π)`. 0은 +X, +π/2는 +Y 방향 |

시간은 Unix 시간이 아니며, 향후 재생 시 시작 시뮬레이션 시각에 더해서 사용할 상대 시간입니다. `yaw`는 수학적인 이동 방향이며, Gazebo actor 모델의 정면 보정은 추후 제어기에서 처리해야 합니다. `z`는 이번 CSV에 포함하지 않습니다.

## 옵션 및 이동 방식

| 옵션 | 기본값 | 의미 |
| --- | --- | --- |
| `--scenario` | `straight` | `straight`, `turn`, `circle`, `square` |
| `--person-id` | `walking_person` | CSV에 기록할 보행자 ID |
| `--duration` | `4` | 전체 이동 시간(s), 양수 |
| `--rate` | `10` | 샘플링 주파수(Hz), 양수 |
| `--start-time` | `0` | 시작 시간 오프셋(s), 0 이상 |
| `--start-x`, `--start-y` | `0`, `0` | 시작 위치(m) |
| `--yaw-deg` | `0` | 시작 이동 방향(deg). CSV에는 rad으로 저장 |
| `--distance` | `2` | 직선 거리(m), 양수 |
| `--radius` | `1` | 회전·원형 경로 반지름(m), 양수 |
| `--side` | `2` | 사각형 경로 한 변의 길이(m), 양수 |
| `--turn-deg` | `90` | `turn`의 회전 각도(deg), 0 제외. 양수 좌회전, 음수 우회전 |
| `--output` | `trajectories/<scenario>.csv` | 출력 경로. 상위 폴더 자동 생성, 기존 파일 덮어쓰기 |

각 경로는 일정한 속도로 이동합니다. 속도는 경로 길이 / 이동 시간으로 결정됩니다. 직선 길이는 `distance`, 원호 길이는 `radius × abs(회전 각도[rad])`, 원형 길이는 `2π × radius`, 사각형 길이는 `4 × side`입니다. 회전은 걸으면서 원호를 따라 방향을 바꾸는 방식입니다. 사각형은 코너에서 이동 방향을 즉시 바꿉니다.

시작점과 종료점을 모두 기록합니다. 예를 들어 4초 × 10 Hz는 헤더를 제외하고 41행입니다. 이동 시간이 샘플 간격으로 나누어떨어지지 않으면 마지막 간격만 짧게 하여 정확한 종료 시각을 포함합니다. 각도 정규화로 인해 ±π 경계에서 `yaw` 숫자가 바뀔 수 있으므로 추후 비교·보간 시 각도 차이도 정규화해야 합니다.

## 추후 여러 보행자 및 ROS 연동

`--person-id human_1 --output trajectories/human_1.csv`처럼 ID와 파일 경로를 지정할 수 있습니다. 같은 방식으로 `human_2`, `human_3`의 궤적도 각각 만들 수 있습니다. ID 지정은 CSV 데이터만 바꾸며 Gazebo actor를 생성하거나 이동시키지 않습니다.

다음 단계의 ROS 발행기나 GUI에서는 CLI 외에 Python API도 사용할 수 있습니다. API의 각도 단위는 rad입니다.

```python
from pathlib import Path
from scripts.generate_trajectory import generate_trajectory, save_csv

points = generate_trajectory(person_id="walking_person", scenario="turn", radius=2)
save_csv(points, Path("trajectories/custom.csv"))

# CSV 저장 없이 샘플 순회도 가능
for point in generate_trajectory(person_id="human_1"):
    print(point.timestamp, point.x, point.y, point.yaw)
```

이번 단계의 결과물은 궤적 생성 및 CSV 저장입니다. CSV를 ROS 2로 발행하는 다음 단계는 [Task 3-2 사용 설명서](trajectory_publisher.md)에 정리했습니다. Gazebo 제어, 실제 위치 기록·비교, GUI는 후속 단계에서 연결합니다.

