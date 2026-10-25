# Task 3-2: ROS 2 궤적 발행기

`scripts/trajectory_publisher.py`는 Task 3-1의 CSV를 읽어서 각 시각의 목표 위치와 방향을 ROS 2로 발행합니다. 한 번 실행하면 CSV 전체를 한 번 재생하고 종료합니다. CSV 내용은 변경하지 않습니다.

## 실행: Gazebo 없이 토픽부터 확인

ROS 2 Humble의 `rclpy`, `geometry_msgs`가 필요합니다. 별도의 colcon 빌드는 필요하지 않습니다. 일반 Ubuntu 시스템 Python인 `python3`를 사용하세요.

터미널 1에서 수신을 먼저 시작합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /humans/walking_person/target_pose geometry_msgs/msg/PoseStamped
```

터미널 2에서 발행기를 실행합니다.

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash

python3 scripts/trajectory_publisher.py \
  --csv trajectories/straight.csv \
  --wait-for-subscribers
```

CSV가 없다면 먼저 생성합니다.

```bash
python3 scripts/generate_trajectory.py \
  --scenario straight --start-x -2 --start-y -2 \
  --distance 4 --duration 4 --rate 10 \
  --output trajectories/straight.csv
```

`--wait-for-subscribers`를 지정하면 모든 보행자의 목표 토픽에 구독자가 연결된 다음 재생을 준비합니다. 기본 시작 지연은 1초입니다. 예제 CSV는 궤적 시간이 4초이고 목표가 41개이므로, 연결 후 약 5초에 걸쳐 모두 발행하고 종료합니다. 다시 보내려면 같은 명령을 다시 실행하면 됩니다. 토픽에는 이력을 보관하지 않으므로, 늦게 연결된 구독자는 이미 발행된 목표를 받지 못합니다.

## 메시지 계약: 다음 단계 제어기와 연결할 형식

| 항목 | 값 |
| --- | --- |
| 토픽 | `/humans/<person_id>/target_pose` |
| 현재 actor의 토픽 | `/humans/walking_person/target_pose` |
| 메시지 형식 | `geometry_msgs/msg/PoseStamped` |
| `header.frame_id` | 기본 `world` |
| `header.stamp` | 재생 기준 ROS 시각 + CSV의 `timestamp` |
| `pose.position.x/y` | CSV의 `x/y`(m) |
| `pose.position.z` | 기본 0(m), `--z`로 변경 가능 |
| `pose.orientation` | CSV의 `yaw`(rad)를 변환한 quaternion |
| QoS | Reliable, Volatile, Keep Last 100 |

`PoseStamped`에는 보행자 ID 필드가 없으므로 ID를 토픽 경로에 넣습니다. `yaw`는 `orientation.z = sin(yaw/2)`, `orientation.w = cos(yaw/2)`로 변환하고 x/y 성분은 0입니다.

[Task 3-3의 Gazebo 제어기](actor_controller.md)는 이 토픽을 구독하여 actor의 위치와 방향을 적용합니다. 발행기만 실행해서는 Gazebo actor가 움직이지 않으며, 제어기 플러그인이 포함된 World를 실행해야 합니다. 제어기는 기존 actor의 높이 1 m를 보존하므로 Publisher의 기본 `z=0`을 그대로 사용하면 됩니다.

## Gazebo 시뮬레이션 시간 사용

Gazebo 실행 후 ROS 2 `/clock`이 전달되는지 먼저 확인합니다. `/clock`이 없다면 시뮬레이션 쪽 ROS bridge 설정이 필요합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /clock rosgraph_msgs/msg/Clock --once
```

시뮬레이션 시간으로 발행하려면 다음처럼 실행합니다.

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash

python3 scripts/trajectory_publisher.py \
  --csv trajectories/turn.csv \
  --wait-for-subscribers \
  --ros-args -p use_sim_time:=true
```

- 기본 모드는 시스템 시간을 사용하므로 Gazebo 없이도 동작합니다.
- `use_sim_time:=true`이면 0보다 큰 `/clock`을 받을 때까지 기다립니다. 이후 시뮬레이션 시간이 멈추면 추가 발행도 멈춥니다. 이 방식은 [ROS 2 시간 규약](https://design.ros2.org/articles/clock_and_time.html)에 따릅니다.
- CSV의 첫 `timestamp`를 0으로 바꾸지 않습니다. 예를 들어 첫 값이 3이면 시작 지연 후 3초를 더 기다립니다.
- 메시지의 stamp는 목표 도달 예정 시각입니다. 실제 콜백 실행 시각과 약간 다를 수 있습니다. 5 ms 주기로 시각을 확인하며, OS 및 DDS 지연이 있으므로 엄격한 실시간 전송을 보장하지 않습니다.
- 시각이 앞으로 크게 이동하면 이미 도래한 목표를 순서대로 모두 발행합니다. 시각이 뒤로 돌아가면 오류로 종료합니다. Gazebo를 리셋한 경우 발행기를 다시 실행하세요.

## 옵션

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--csv` | 필수 | 입력 CSV 경로 |
| `--start-delay` | `1.0` | 재생 기준 시각까지의 시작 지연(s), ROS 시간 기준 |
| `--frame-id` | `world` | 좌표계 이름. 이름만 지정하며 좌표 변환은 수행하지 않음 |
| `--z` | `0.0` | 목표 높이(m) |
| `--wait-for-subscribers` | 꺼짐 | 모든 보행자 토픽에 구독자가 연결될 때까지 시작 대기 |
| `--experiment-dir` | 미지정 | 실행 중인 Recorder의 실험 폴더. Target 사본과 재생 시작 시각을 저장하여 GT와 비교 |
| `--ros-args -p use_sim_time:=true` | 꺼짐 | ROS 시뮬레이션 시간 사용 |

CSV 헤더는 `person_id,timestamp,x,y,yaw`의 다섯 열을 포함해야 합니다. 시간은 0 이상인 상대 초이며, 각 보행자의 시간이 엄격하게 증가해야 합니다. 비어 있는 CSV, 누락된 열, 잘못된 숫자, NaN/무한대, 중복·역순 시각은 시작 전에 거부합니다. ID는 ROS 토픽에 사용할 수 있도록 영문자 또는 `_`로 시작하고 영문자·숫자·`_`만 허용합니다.

나중에 ID를 `human_1`로 생성하면 토픽은 `/humans/human_1/target_pose`가 됩니다. 한 CSV 안에 여러 ID가 있으면 각 ID별 토픽으로 같은 재생 시작 시각을 기준으로 발행합니다. 보행자별 CSV 파일을 별도로 실행하면 각 프로세스의 재생 시작 시각은 서로 다를 수 있습니다.

