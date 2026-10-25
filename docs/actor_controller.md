# Task 3-3: Gazebo Fortress 보행자 제어기

Task 3-1/3-2의 CSV와 Publisher를 그대로 사용합니다. 제어기는 별도 Python 프로세스가 아니라 **Gazebo 서버에 로드되는 C++ System Plugin**입니다. Gazebo Fortress의 `ignition::gazebo::Actor` API를 사용하며 Classic의 `gazebo::physics::Actor`를 사용하지 않습니다.

```text
generate_trajectory.py → CSV → trajectory_publisher.py
    → geometry_msgs/msg/PoseStamped
    → /humans/human_1/target_pose
    → Gazebo 내 human_actor_controller 노드
    → world 좌표를 trajectory origin 기준 상대 좌표로 변환
    → Actor::SetTrajectoryPose() → walking_person
```

## 기존 코드와 매핑

- 기존 Publisher의 토픽: `/humans/<CSV person_id>/target_pose`
- 메시지: `geometry_msgs/msg/PoseStamped`, `header.frame_id=world`
- 기본 논리 ID: `human_1` → Gazebo Actor `walking_person`
- 기존 CSV의 `walking_person` ID도 `/humans/walking_person/target_pose`로 입력 가능
- 두 토픽은 같은 actor를 제어하므로 한 번에 하나의 Publisher만 사용하세요.

기존 Generator와 Publisher의 기본 ID를 변경하지 않았습니다. 새 궤적을 만들 때 `--person-id human_1`을 명시하면 기본 논리 ID를 사용할 수 있습니다.

## 파일 구성과 SDF 변경 이유

| 파일 | 역할/변경 이유 |
| --- | --- |
| `ros2/jackal_actor_controller/src/ActorController.cpp` | ROS 수신, 명령 스케줄링, 좌표 변환, Actor/애니메이션 제어 |
| `ros2/jackal_actor_controller/package.xml` | 별도 ROS 2 패키지 의존성 |
| `ros2/jackal_actor_controller/CMakeLists.txt` | Fortress 플러그인 빌드·설치·통합 테스트 |
| `ros2/jackal_actor_controller/env-hooks/actor_controller.dsv.in` | source 시 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`에 플러그인 경로 추가 |
| `worlds/jackal_actor_world.sdf` | World에 Controller 플러그인과 ID/Actor/토픽 매핑만 추가 |
| `scripts/setup_actor_controller.sh` | 소스 패키지와 World를 기존 워크스페이스에 심볼릭 링크로 연결 |
| `scripts/setup_workspace.sh` | 최초 설치에도 동일한 링크 설정 적용 |
| `README.md`, 이 문서 | 설치·실행·검증 설명 |

기존 SDF의 actor 한 명, mesh, walk 애니메이션, 사각형 waypoint, 물리·센서 플러그인, 장애물은 유지합니다. `SetTrajectoryPose()`가 SDF 경로를 덮어쓰므로 자동 이동 스크립트를 삭제할 필요가 없습니다. 플러그인은 첫 업데이트부터 시작 waypoint의 위치를 지정하여 외부 명령 전에도 사각형 이동을 막습니다. 명령 종료 시 마지막 위치를 유지합니다.

### 화면에서 정지해 있던 문제의 수정

Fortress 6의 `Actor::SetTrajectoryPose()`와 `SetAnimationTime()`은 컴포넌트 값만 수정하고, 이후 변경을 GUI에 알리는 표시를 자동으로 갱신하지 않습니다. 이전 코드는 서버 내부 위치는 바뀌지만 SceneBroadcaster를 통해 화면으로 위치·애니메이션 변경이 계속 전달되지 않는 문제가 있었습니다.

Controller에서 `TrajectoryPose`와 `AnimationTime`을 `ecm.SetChanged(..., ComponentState::PeriodicChange)`로 표시하도록 수정했습니다. 수정 당시 서버 내부 값과 함께 GUI용 상태 메시지에 위치·방향과 진행된 애니메이션 시간이 포함되는 것을 확인했습니다.

새 라이브러리는 실행 중인 Gazebo에 자동 교체되지 않으므로 **기존 Gazebo를 종료한 후 빌드·소싱하고 다시 실행**해야 합니다. 첫 명령 적용 시 서버 터미널에 다음 로그가 표시됩니다.

```text
External control active: human_1 -> walking_person; world target x=-2.000 y=-2.000 yaw=0.000
```

## 빌드와 실제 로드 경로

이 환경에서 기존 Clearpath launch는 다음 파일을 로드합니다.

```text
~/jackal_actor_ws/install/clearpath_gz/share/clearpath_gz/worlds/jackal_actor_world.sdf
  → ~/jackal_actor_ws/src/clearpath_simulator/clearpath_gz/worlds/jackal_actor_world.sdf
```

기존 파일은 저장소 World의 복사본이었습니다. 연결 스크립트는 복사본을 `.before_actor_controller` 파일로 백업한 뒤 저장소 원본을 가리키는 링크로 바꿉니다. 이미 올바른 링크가 있으면 그대로 유지하고, 다른 패키지나 기존 백업을 덮어쓰지 않습니다.

현재 실행 중인 Gazebo가 있다면 종료한 후 아래 명령을 실행하세요.

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash

bash scripts/setup_actor_controller.sh ~/jackal_actor_ws

cd ~/jackal_actor_ws
colcon build --symlink-install --packages-select jackal_actor_controller
source install/setup.bash
```

필요 개발 의존성은 `rclcpp`, `geometry_msgs`, `libignition-gazebo6-dev`, `libignition-plugin1-dev`이며 현재 환경에서는 설치되어 있습니다. 의존성이 없는 새 환경에서는 패키지 매니저로 설치해야 합니다.

빌드 후 실제 로드 경로를 확인하세요.

```bash
ros2 pkg prefix --share clearpath_gz
readlink -f ~/jackal_actor_ws/install/clearpath_gz/share/clearpath_gz/worlds/jackal_actor_world.sdf
ros2 pkg prefix jackal_actor_controller
printenv IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

World의 최종 경로가 `/home/sm/colcon_ws/jackal_actor_sim/worlds/jackal_actor_world.sdf`이어야 합니다. `clearpath_gz`가 복사 설치된 다른 환경이면 `colcon build --symlink-install --packages-select clearpath_gz`도 실행하여 설치된 World를 갱신하세요. 여러 overlay를 사용하는 경우 `ros2 pkg prefix --share clearpath_gz` 결과가 실제로 사용하는 워크스페이스인지 확인해야 합니다.

## 실행 순서

**터미널 1: 기존 방식으로 시뮬레이션 실행**

```bash
source /opt/ros/humble/setup.bash
source ~/jackal_actor_ws/install/setup.bash

ros2 launch clearpath_gz simulation.launch.py \
  world:=jackal_actor_world x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 rviz:=false
```

Gazebo 서버 로그에서 다음 내용을 확인하세요. Controller를 별도로 `ros2 run`할 필요는 없습니다.

```text
/humans/human_1/target_pose -> walking_person (human_1)
/humans/walking_person/target_pose -> walking_person (human_1)
Bound human_1 to entity ...; preserved world z=1.000
```

**터미널 2: 연결 확인 후 궤적 생성·발행**

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash
source ~/jackal_actor_ws/install/setup.bash

ros2 node info /human_actor_controller
ros2 topic info /humans/human_1/target_pose --verbose
ros2 topic echo /clock rosgraph_msgs/msg/Clock --once

python3 scripts/generate_trajectory.py \
  --person-id human_1 --scenario straight \
  --start-x -2 --start-y -2 --distance 4 --duration 4 --rate 10 \
  --output trajectories/human_1_straight.csv

python3 scripts/trajectory_publisher.py \
  --csv trajectories/human_1_straight.csv \
  --wait-for-subscribers \
  --ros-args -p use_sim_time:=true
```

직선이라면 `(-2,-2)`에서 `(2,-2)`로 이동하고 멈춰야 합니다. 이후 북쪽으로 돌아 사각형 경로를 계속 걷지 않아야 합니다. 기존 `trajectories/straight.csv`도 그대로 발행할 수 있으며, 그 경우 구독 토픽은 기존 ID인 `/humans/walking_person/target_pose`입니다.

회전도 확인하려면 다음 CSV를 생성한 뒤 같은 Publisher로 발행합니다.

```bash
python3 scripts/generate_trajectory.py \
  --person-id human_1 --scenario turn \
  --start-x -2 --start-y -2 --radius 2 --turn-deg 90 --duration 4 \
  --output trajectories/human_1_turn.csv

python3 scripts/trajectory_publisher.py \
  --csv trajectories/human_1_turn.csv --wait-for-subscribers \
  --ros-args -p use_sim_time:=true
```

## 좌표·시간·애니메이션

Fortress의 렌더러는 `worldPose = actor.Pose() × trajectoryPose`로 계산합니다. 따라서 제어기는 `relative = origin.Inverse() × desiredWorldPose`를 `SetTrajectoryPose()`에 전달합니다. 원점이 이동하거나 회전한 경우도 고려하며, actor의 origin 자체는 바꾸지 않습니다. 근거: [Fortress Actor API 구현](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/Actor.cc), [Fortress 렌더링 구현](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/rendering/RenderUtil.cc).

- x/y와 yaw는 World 좌표로 적용합니다. `world` 이외 frame의 명령은 거부하며 TF 변환을 수행하지 않습니다.
- z, roll, pitch는 기존 actor의 시작 World Pose에서 보존합니다. 현재 actor는 첫 waypoint의 z=1 m를 유지하므로 Publisher의 기본 z=0 때문에 지면 아래로 내려가지 않습니다.
- `yaw_offset`(rad)은 mesh 정면 보정용입니다. 현재 기본은 기존 SDF와 같은 0이며, GUI에서 모델 정면을 확인한 뒤 필요한 경우 조정할 수 있습니다.
- 애니메이션은 기존 이름 `walk`를 사용합니다. 첫 목표로의 배치는 순간 이동으로 처리하고, 이후 위치 변경 명령이 계속 들어오는 동안 시뮬레이션 dt로 `SetAnimationTime()`을 진행합니다. 위치 명령이 멈추면 기본 0.3초 후 마지막 걷기 프레임에서 정지합니다. 별도 idle 애니메이션을 추가하지 않습니다.
- Controller는 Gazebo의 `UpdateInfo.simTime`을 사용합니다. Publisher는 `use_sim_time:=true`로 `/clock`을 사용해야 같은 시간축이 됩니다. Controller의 ROS 노드 자체는 `/clock`을 다시 구독하지 않습니다.
- 미래 시각의 명령은 대기열에서 기다리고, 해당 시뮬레이션 시각이 되면 적용합니다. stamp=0 수동 명령은 다음 unpaused 업데이트에서 적용합니다.
- Gazebo 일시정지 중에는 목표 적용과 애니메이션 진행이 멈춥니다. 시뮬레이션 시각이 뒤로 돌아가면 대기열을 비우고 시작 위치로 돌아갑니다. 이때 Publisher를 다시 실행하세요.
- 현재 10 Hz 목표는 도착한 샘플의 위치를 유지하는 방식으로 적용합니다. 샘플 사이의 위치 보간·장애물 회피·동역학 제어는 하지 않습니다. 더 촘촘한 위치 갱신이 필요하면 Generator의 `--rate`를 높일 수 있습니다.
- 프레임 오류, 유효하지 않은 숫자/quaternion, 역순 목표, 1,000개 초과 대기열을 거부합니다. 현재 시각보다 60초 이상 미래의 명령은 시스템 시간 사용 오류일 가능성이 있어 거부합니다.

## 나중에 세 명으로 확장

Task 2에서 실제 actor를 추가한 뒤 World의 Controller 플러그인 안에 매핑을 추가하면 됩니다. 현재는 이 actor들을 추가하지 않습니다.

```xml
<mapping>
  <person_id>human_2</person_id>
  <actor_name>human_2</actor_name>
  <topic>/humans/human_2/target_pose</topic>
  <animation>walk</animation>
  <yaw_offset>0</yaw_offset>
</mapping>
```

`human_3`도 같은 방식으로 추가합니다. 각 actor는 독립된 입력 토픽, 대기열, 마지막 목표, 애니메이션 시간을 갖습니다. 실제 actor 이름과 애니메이션 이름은 Task 2 결과와 맞춰야 합니다.

## 검증 방법과 확인 범위

시뮬레이션 실행에 필요하지 않은 자동 테스트 소스는 사용자 요청으로 삭제했습니다. 아래 표는 삭제 전에 수행한 검증 이력입니다. 실제 궤적 비교는 [Ground Truth Recorder와 Validator](ground_truth_validation.md)를 사용합니다.

### 이번 환경에서 실제 수행한 결과 (2026-10-25)

| 검증 | 결과 |
| --- | --- |
| 실제 `~/jackal_actor_ws`에서 colcon 빌드·설치 | 성공. Fortress 6.18.0 / ROS 2 Humble |
| C++ 통합 테스트 6개 | 모두 통과. CTest 1개 안에서 gtest 6개 실행 |
| ROS 구독·기본 ID와 기존 토픽 호환 | DDS 통합 테스트에서 확인 |
| 이동·회전된 trajectory origin의 좌표 변환 | x/y/yaw와 원점 불변 확인 |
| 높이 유지와 SDF 자동 경로 덮어쓰기 | z=1 유지, SDF waypoint 시간이 지난 뒤에도 외부 목표 유지 확인 |
| 예약 시각·일시정지·애니메이션 시간 | 서버의 trajectory/animation 컴포넌트에서 확인 |
| GUI 전달 경로 | 회귀 검사 통과. 2개 위치·방향 목표 `(0,0,yaw=0.2)`, `(0.1,0.2,yaw=0.7)` 및 진행된 애니메이션 시간이 SceneBroadcaster의 실제 상태 메시지로 전달됨 |
| 잘못된 frame·시스템 시각 명령 | 거부 동작 확인 |
| 실제 설치 World + `/clock` bridge + 기존 Python Publisher | 직선 CSV 41개 발행 완료, 실제 서버의 actor trajectory 최종 World Pose가 `(2, -2, 1, yaw=0)`인 것을 state 서비스로 읽어 확인 |
| GUI의 렌더링 위치·mesh 정면·실제 걷기 동작 | 미확인 |
| Jackal·LiDAR·Camera·Teleop 전체 실행 회귀 검사 | 미확인. 기존 구성과 플러그인은 변경하지 않았으나 정상 동작을 새로 검증한 것은 아님 |
| 세 actor 동시 제어 | 매핑 확장 구조만 구현. actor 추가 및 동시 실행 미검증 |

서버 모드 연결 검사에서는 기존 Publisher 종료 시 `Timed out waiting for subscriber acknowledgment.` 경고가 발생했습니다. 이 경고는 완전히 해소하지 않았습니다. 별도로 Gazebo state 서비스를 읽어 마지막 목표까지 Actor trajectory에 반영된 것을 확인했으며, 명령 누락으로 단정하지 않았습니다. 실제 화면이나 센서 검증을 이 결과로 대신하지 않습니다. 해당 검사에서 시작한 서버와 bridge는 종료했습니다.

GUI에서 추가로 확인할 사항:

1. 발행 전 보행자가 시작 위치에서 대기하는지
2. 직선·회전 CSV에 따라 위치와 방향이 바뀌는지, 재생 후 위치를 유지하는지
3. 보행자가 지면 높이를 유지하며 걷기 동작을 하는지, 정면 방향 보정이 필요한지
4. Gazebo 일시정지 시 보행자와 Publisher의 진행이 함께 멈추는지
5. 기존 Teleop으로 Jackal이 이동하는지
6. `/j100_0000/sensors/lidar3d_0/points`, `/j100_0000/sensors/camera_0/color/image`, `/j100_0000/sensors/camera_0/points`가 계속 발행되는지

```bash
ros2 topic hz /j100_0000/sensors/lidar3d_0/points
ros2 topic hz /j100_0000/sensors/camera_0/color/image
ros2 topic hz /j100_0000/sensors/camera_0/points
```

이 문서의 Task 3-3 범위는 Actor 제어입니다. 이후 추가된 기능은 [Task 3-4 GT 기록·비교](ground_truth_validation.md)와 [Task 3-5 경로 GUI](trajectory_gui.md)를 참고하세요.
