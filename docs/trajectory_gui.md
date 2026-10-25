# Task 3-5: 마우스로 그리는 보행자 경로 GUI

기존 `trajectory_viewer.py`에 PyQt5/Matplotlib 경로 편집 창을 연결했습니다. Qt Creator나 새 ROS 패키지 없이 기존 Publisher, Actor Controller, GT Recorder를 재사용합니다.

## Gazebo와 같은 방향으로 보기

기본 **XY view: Gazebo default: +X up**은 현재 Clearpath 실행 설정의 카메라(`camera_pose=-6 0 6 0 0.5 0`, yaw=0)에 맞춰 **+X를 위쪽, +Y를 왼쪽**으로 표시합니다. 이전 GUI는 +X가 오른쪽이고 +Y가 위쪽인 일반 XY 보기였으므로 Gazebo의 화면 방향과 달랐습니다.

| Gazebo 기본 보기에서 그리는 방향 | 실제 World 좌표 변화 |
|---|---|
| 위 | X 증가 |
| 아래 | X 감소 |
| 왼쪽 | Y 증가 |
| 오른쪽 | Y 감소 |

그리기 창과 메인 Target/GT 비교 그래프에 같은 방향을 적용합니다. 축 라벨, 빨간 +X·초록 +Y 방향 표시, 마우스 커서의 World 좌표를 함께 표시합니다. 좌표 표와 CSV, ROS 목표 메시지, GT, 시간별 X/Y 그래프는 모두 실제 World X/Y를 사용합니다. 기본 보기에서 가로축의 `-World Y`는 화면 오른쪽이 World -Y 방향임을 의미합니다.

**World XY: +Y up**은 이전 일반 XY 보기(+X 오른쪽, +Y 위쪽)입니다. 반대·측면 보기와 임의 각도도 선택할 수 있습니다. 숫자 각도는 카메라가 수평으로 바라보는 World yaw(deg)에 해당합니다. Gazebo에서 카메라를 회전했다면 방향 표시를 보면서 이 값을 조절하세요. 카메라 자세를 자동 추적하지 않으며, GUI는 기울기·원근감이 없는 2D 평면 보기입니다.

내부적으로 World `(x,y)`를 표시 좌표 `(sin(yaw)*x-cos(yaw)*y, cos(yaw)*x+sin(yaw)*y)`로 회전하고, 마우스 입력은 그 역변환으로 World 좌표에 저장합니다. 기존 CSV를 다시 회전해 저장하거나 Controller에 중복 변환하지 않습니다. 표시 방향을 바꾸는 것만으로는 궤적·yaw·재생 시각·오차 계산이 변경되지 않습니다.

## 실행

기존 Task 3-4의 GT 기능이 있는 Controller를 설치한 환경에서는 Python 변경에 대한 추가 빌드가 필요하지 않습니다. Controller를 아직 빌드하지 않았다면 Gazebo를 종료한 상태에서 먼저 실행합니다.

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash
bash scripts/setup_actor_controller.sh ~/jackal_actor_ws
cd ~/jackal_actor_ws
colcon build --symlink-install --packages-select jackal_actor_controller
```

터미널 1에서 Gazebo를 실행하고 재생 상태로 둡니다.

```bash
cd ~/jackal_actor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch clearpath_gz simulation.launch.py \
  world:=jackal_actor_world x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 rviz:=false
```

터미널 2에서 GUI를 실행합니다. CSV 없이 시작할 수 있습니다.

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash
source ~/jackal_actor_ws/install/setup.bash
python3 scripts/trajectory_viewer.py --start-delay 3
```

기존 CSV로 시작하려면 `--csv trajectories/human_1_right_turn.csv`를 추가합니다. 시뮬레이션 없이도 경로를 그리고 저장할 수 있고, 전송은 `/clock`, 실제 GT, Controller 구독 연결을 기다립니다. 별도 Publisher나 Recorder를 동시에 실행할 필요가 없습니다.

## 경로 그리기와 수정

1. 메인 창에서 **Draw / edit path**를 누릅니다.
2. 새로 그리려면 **Clear**를 누릅니다. 기존 CSV가 있으면 현재 경로를 점 목록으로 불러옵니다.
3. **Mouse mode: Draw / append**에서 왼쪽 클릭으로 점을 추가하거나 드래그해서 선을 그립니다. 기존 끝점에서 계속 이어집니다. 별은 시작점이고 삼각형은 종료점입니다.
4. **Mouse mode: Move waypoints**에서 점을 잡아 옮깁니다. 점 근처에서 오른쪽 클릭하거나 표의 행을 선택해 **Delete selected**로 삭제할 수 있습니다.
5. 표의 X/Y 값을 더블클릭하면 정확한 좌표로 수정할 수 있습니다. **Undo**, **Redo**, `Ctrl+Z`, `Ctrl+Shift+Z`도 지원합니다. 드래그 한 번을 하나의 되돌리기 단위로 취급합니다.
6. **Walking speed**와 **Sample rate**를 설정합니다. 경로 길이와 예상 이동 시간이 표시됩니다.
7. **Use path / preview**는 메인 창에 목표 경로를 적용하고 실제 이동은 시작하지 않습니다. 메인 창에서 **Start trajectory**를 눌러 전송합니다. **Send to Gazebo**는 경로 적용 후 전송 준비까지 한 번에 진행합니다.

기본 편집 구역은 `x,y ∈ [-2,2] m`입니다. **Limit x and y to [-2, 2] m**를 해제하면 구역 밖의 좌표도 사용할 수 있습니다. **Fit view**는 전체 경로와 기준 4×4 m 영역이 보이도록 화면 범위를 맞춥니다. 좌표는 Gazebo World 기준 m입니다. 장애물 지도나 충돌 회피는 포함하지 않습니다.

편집 창의 **Save CSV**는 현재 선택한 사람의 궤적을 저장합니다. 메인 창의 **Save target CSV**는 현재 전송 대상 전체를 저장합니다. 파일명은 직접 선택하며, 기존 파일 선택 시 저장 대화상자가 덮어쓰기를 확인합니다.

## 궤적 변환

마우스로 입력한 점들의 누적 거리를 계산하고, `timestamp = 이동 거리 / 속도`로 도착 시간을 정합니다. 설정한 주기의 샘플 외에 각 꼭짓점 도착 시각도 포함해 꺾이는 지점을 보존합니다. 따라서 CSV 샘플 개수는 단순한 `시간×Hz+1`보다 많을 수 있습니다.

- CSV 형식: `person_id,timestamp,x,y,yaw`.
- timestamp: 재생 시작 기준 상대 초, 첫 샘플은 0초.
- yaw: 각 선분의 이동 방향 `atan2(dy, dx)`, 마지막 샘플은 마지막 선분 방향 유지.
- 속도: 경로를 따라 일정하게 설정. 곡선 맞춤이나 코너 감속은 적용하지 않습니다.
- 연속 중복점은 제거하고, 폐곡선의 시작·종료점은 유지합니다.
- 최대 2,000개 편집점, 최대 50,000개 생성 샘플, 이동 시간 최대 3,600초입니다. 제한 초과는 화면에 오류로 표시합니다.
- CSV를 편집하면 위치 경로를 선택한 속도로 다시 시간화하고 yaw를 이동 방향으로 계산합니다. 원본 CSV의 대기 시간이나 별도 방향 명령은 편집 시 유지하지 않습니다. 원본 파일은 별도로 저장하기 전까지 변경하지 않습니다.

현재 기본 매핑은 `human_1=walking_person`입니다. 추가 매핑은 기존 `--mapping` 옵션을 사용합니다. 여러 사람의 CSV가 있는 경우 선택한 한 사람의 경로를 교체하고 나머지 사람의 목표 샘플은 유지합니다. 전송 시 같은 재생 시작 시각을 사용합니다. 편집 창에서 Use/Send는 현재 선택한 사람에만 적용됩니다. 실제 세 Actor 환경의 동작 검증은 아직 수행하지 않았습니다.

## 전송과 실제 경로 확인

```text
마우스/좌표 편집
  → trajectory_path.py: 상대 시간·XY·yaw 계산
  → trajectory_viewer.py: 목표 CSV 스냅샷 고정
  → 기존 trajectory_publisher.py: /humans/human_1/target_pose
  → 기존 ActorController.cpp: walking_person 제어
  → 실제 Actor WorldPose 관측: /humans/human_1/ground_truth
  → 기존 Recorder/LiveTracks: 기록·시간 정렬·보간·오차 계산
  → 메인 UI의 XY·시간별 X/Y·거리 오차 그래프
```

전송 준비 중과 재생 중에는 편집·CSV 교체를 잠급니다. **Stop publishing**은 추가 명령 전송을 중단하며 Controller는 마지막으로 받은 위치를 유지합니다. 시작점과 기존 Actor 위치가 다르면 첫 명령에서 시작점으로 이동합니다. 시뮬레이션을 리셋했다면 새 실험을 시작합니다.

## 기록

그린 경로를 적용하면 `results/live/run_<UTC시각>_<식별자>/` 아래에 저장합니다.

- `drawn_path.csv`: 전송 대상으로 생성한 목표 샘플. 여러 사람이 있으면 전체 목표 포함.
- `drawn_path.json`: 사람별 편집점, 속도, 샘플링 주기, Actor 매핑.
- `input.csv`, `target.csv`, `playback.json`: 실제 재생에 사용한 고정된 CSV와 정확한 시작 시각. Publisher가 시작된 뒤 생성.
- `ground_truth.csv`: 실제 수신한 GT만 기록. GT가 없으면 파일을 만들지 않음.
- `live_comparison.csv`, `live_metrics.csv`, `live_metadata.json`: 기존 실시간 보간·비교 결과.

PNG 저장 기능은 추가하지 않았습니다. 반복 재생은 새 실험 폴더를 사용합니다. `drawn_path.json`은 편집 설정 기록용이며, 현재 다시 불러오기 인터페이스는 CSV를 사용합니다. 같은 창에서는 원래 편집점을 유지해 다시 수정할 수 있습니다.

## 구현 파일과 검증

- `scripts/trajectory_editor.py`: 경로 편집 창, 마우스/좌표 수정, Undo/Redo, CSV 저장.
- `scripts/trajectory_path.py`: 누적 거리 기반 시간·yaw 계산, 기존 Task 3-1 데이터 구조와 CSV 규약 재사용.
- `scripts/trajectory_viewer.py`: 빈 상태로 시작, 편집 창 연결, 목표 저장, 기존 Publisher·GT 비교 연결.
- `scripts/trajectory_view_frame.py`: Gazebo 시점에 맞춘 표시 회전과 마우스 역변환, 축 라벨·방향 표시.

실제 창 조작과 별도 Fortress 서버의 한 Actor로 검증했습니다. 마우스로 생성·수정한 2 m 꺾인 경로를 GUI로 전송해 실제 Actor 이동과 GT 기록을 확인했습니다. 상세 결과와 범위는 `results/task35_real_01/REPORT.md`에 있습니다. 전체 Jackal의 LiDAR·Camera·Teleop 동시 실행 및 세 Actor 통합은 이번 검증 범위에 포함하지 않았습니다. 테스트 소스는 저장소에 추가하지 않았습니다.

보기 방향 수정 후에도 실제 UI에서 +X 위쪽·+Y 왼쪽 입력, 점 이동·삭제, CSV 보존, 실제 Gazebo Actor 이동과 GT 관측을 재검증했습니다. 결과는 `results/task35_axes_01/REPORT.md`에 있습니다.
