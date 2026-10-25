# 실시간 궤적 UI와 Ground Truth 비교

`trajectory_viewer.py`가 별도의 Python 창에서 Target CSV와 실제 Gazebo 보행자의 경로·오차를 실시간으로 표시합니다. UI 안에서 기존 Publisher를 시작하고 실제 GT를 CSV로 기록합니다. PNG 저장 기능과 기존 경로·오차 PNG 기록은 삭제했습니다. README의 Gazebo/RViz 소개 이미지는 유지합니다.

Task 3-5에서 같은 창에 경로 그리기·수정·CSV 저장 기능을 추가했습니다. CSV 없이 실행하는 방법과 마우스 편집 방법은 [보행자 경로 GUI](trajectory_gui.md)를 참고하세요.

## 실행 순서

이 변경은 Python 코드이므로 추가 colcon 빌드는 필요하지 않습니다. Task 3-4의 GT 관측 기능이 포함된 Controller를 빌드·설치한 환경을 사용합니다.

### 터미널 1: Gazebo

```bash
cd ~/jackal_actor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch clearpath_gz simulation.launch.py \
  world:=jackal_actor_world x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 rviz:=false
```

Gazebo를 재생하고 Jackal 및 센서가 준비될 때까지 기다립니다.

### 터미널 2: CSV 생성 후 실시간 UI

```bash
cd /home/sm/colcon_ws/jackal_actor_sim
source /opt/ros/humble/setup.bash
source ~/jackal_actor_ws/install/setup.bash

python3 scripts/generate_trajectory.py \
  --person-id human_1 --scenario turn \
  --start-x -2 --start-y 2 --radius 2 --turn-deg -90 \
  --duration 4 --rate 10 \
  --output trajectories/human_1_right_turn.csv

python3 scripts/trajectory_viewer.py \
  --csv trajectories/human_1_right_turn.csv \
  --start-delay 3
```

창에서 **Start trajectory**를 누르면 Controller의 실제 GT와 `/clock`이 준비된 뒤 궤적을 전송합니다. Recorder와 Publisher를 별도 터미널에서 실행할 필요가 없습니다. 같은 보행자에 다른 Publisher를 동시에 실행하지 마세요.

- **Open CSV**: 다른 Target CSV 선택.
- **Start trajectory**: 선택한 CSV 재생. 다시 누르면 새 실험 폴더에서 재기록·재생.
- **Stop publishing**: 위치 명령 전송 중지. 실제 GT 관측은 계속 표시.
- **Target: linear**: CSV 샘플 사이를 선형 보간한 연속 목표 경로와 비교.
- **Target: hold samples**: 현재 Controller처럼 다음 목표까지 이전 위치를 유지하는 방식으로 비교.
- 그래프 툴바: 확대·이동·초기 보기. 이미지 저장 버튼과 저장 단축키는 제거했습니다.

## 화면과 보간의 의미

왼쪽 그래프는 Target CSV 경로(빨간 선), 실제 측정된 GT(검은 점), 측정 사이를 선형 보간한 GT(검은 선)를 동일한 XY 평면에 겹쳐 표시합니다. XY 단위는 m이고 축 스케일은 같습니다. 시작 지점은 별, 목표 종료 지점은 빨간 삼각형, 실제 종료 지점은 검은 삼각형으로 구분합니다. 실제 종료 표시는 전체 시간 범위의 GT가 수집된 뒤 나타납니다. 현재 비교 시각의 목표·GT 위치 사이를 회색 점선으로 연결합니다.

기본 표시 방향은 Gazebo 초기 카메라에 맞춰 +X 위쪽·+Y 왼쪽입니다. **XY view**로 표시 방향을 바꿀 수 있으며, CSV·GT·시간별 X/Y·오차는 계속 World 좌표를 사용합니다. 마우스 커서의 좌표도 World 값입니다. 카메라를 회전한 경우의 조정 방법은 [경로 GUI의 보기 방향 안내](trajectory_gui.md)를 참고하세요.

오른쪽 위·가운데 그래프는 시간별 Target과 GT의 X·Y 위치(m)를 각각 비교합니다. 오른쪽 아래 그래프는 같은 시각의 두 위치 사이 거리 `sqrt((x_GT-x_target)^2 + (y_GT-y_target)^2)`를 표시합니다. 시간축은 재생 시작 시각 기준 초(s)입니다.

XY 그래프 안에는 평균 위치 오차, RMSE, 목표 경로 전체 길이와 현재까지 비교한 GT 경로 길이를 표시합니다. 창 아래에는 현재 비교 시각과 거리 오차, Mean/RMSE/Max 위치 오차(m), 평균 절대 yaw 오차(rad), 보간 지점 수, 실제 관측 수가 표시됩니다.

GT 선형 보간은 **두 실제 관측점이 확보된 구간**을 채웁니다. 관측이 끊긴 동안에는 마지막 관측 위치까지만 표시하고, 다음 실제 값이 오면 그 사이를 채웁니다. 큰 간격도 양 끝 관측이 있으면 보간합니다. 아직 측정이 없는 미래로 외삽하지 않습니다.

첫 목표 명령은 대기 위치에서 시작점으로 순간 이동시킬 수 있으므로, 재생 전 대기 위치와 첫 목표점 사이의 순간 이동은 보행 경로 보간에 포함하지 않습니다. 비교는 재생 구간 내 첫 실제 GT 관측부터 시작합니다.

yaw는 ±π 경계를 고려한 최단 각도로 보간·비교합니다. 기본 보간 격자는 50 Hz이며 `--interpolation-rate`로 바꿀 수 있습니다. 긴 실험의 화면 계산은 보행자별 최대 5,000지점으로 제한하고 실제 관측의 메모리 보관은 최근 20,000개로 제한합니다. 원본 GT CSV에는 수신한 전체 관측을 그대로 기록합니다.

보간 지점은 실제 관측이 아니라 두 관측 사이의 추정값입니다. UI 지표와 `live_metrics.csv`는 이 보간 격자를 기준으로 계산하며, 기존 관측 샘플 기준의 오프라인 지표와 값이 다를 수 있습니다. 화면에는 실제 관측점과 보간 선을 구분하고 CSV에도 보간 여부를 남깁니다.

## 데이터 수집원과 시간 정렬

Fortress의 `/world/jackal_actor_world/pose/info`, `dynamic_pose/info`에 `walking_person` 이동 위치가 없는 것을 실행 중 확인했습니다. 기존 Controller는 실제 렌더링 `SceneUpdate`에서 Actor Visual의 `WorldPose()`와 해당 Scene의 `Time()`을 읽어 `/humans/human_1/ground_truth`에 `PoseStamped`로 발행합니다. 목표 명령을 GT로 재사용하지 않습니다.

GT timestamp는 `/clock`과 같은 시뮬레이션 절대 시간입니다. Target CSV timestamp는 상대 시간이며, UI가 실행한 Publisher가 `playback.json`에 정확한 재생 시작 시각을 저장합니다. 임의로 첫 관측점을 0초에 맞추지 않습니다.

Recorder는 기본적으로 기존 `/j100_0000/sensors/camera_0/color/image`를 읽어 lazy 렌더링 센서가 동작하게 합니다. 이미지는 GT 계산에 사용하지 않습니다. 다른 센서가 렌더링을 유지하는 경우 `--keepalive-image-topic ''`로 이 구독을 끌 수 있습니다. 실제 프레임이 없으면 GT도 생성되지 않습니다.

시뮬레이션을 일시정지하면 관측 시간도 멈춥니다. 시뮬레이션 시각을 리셋한 경우 새 Start 세션을 사용합니다. GT가 잠시 끊겨도 UI는 다음 실제 관측을 기다리며, 원본 CSV에 임의 값을 채워 넣지 않습니다.

## 저장 파일

기본 출력은 `results/live/run_<UTC시각>_<식별자>/`입니다. `--output-root`로 상위 폴더를 바꿀 수 있습니다. 재생마다 새 폴더를 사용합니다.

```text
results/live/run_.../
├── ground_truth.csv
├── ground_truth_metadata.json
├── input.csv
├── target.csv
├── playback.json
├── publisher.log
├── live_comparison.csv
├── live_metrics.csv
└── live_metadata.json
```

- `ground_truth.csv`: 실제 관측만 기록. `person_id,timestamp,x,y,yaw`.
- `ground_truth_metadata.json`: GT 수집원·좌표계·시간 기준·Actor 매핑·완료/실패 상태.
- `input.csv`, `target.csv`: 선택한 CSV의 재생용 스냅샷과 Publisher가 실제 사용한 사본.
- `playback.json`: 정확한 재생 시작 시각, 실험 ID, Target 해시.
- `live_comparison.csv`: 같은 시각의 Target/GT XY·yaw, 오차, `gt_interpolated` 표시.
- `live_metrics.csv`: 보간 비교 격자 기준 요약 지표.
- `live_metadata.json`: 보간 방식·격자 빈도·메모리 제한·원본 파일 정보.

비교 CSV는 재생 완료와 창 종료 시 저장됩니다. 관측이나 시간 정렬이 준비되지 않으면 비교 결과를 만들지 않습니다. PNG는 생성하지 않습니다.

## 창 없이 기록·CSV 지표만 계산

기존 Recorder와 숫자 검증기도 유지합니다. 아래 작업은 UI와 별도로 새로운 실험에서 수행합니다.

```bash
# 먼저 실행하고, 다른 터미널에서 해당 실험 폴더로 Publisher 실행
python3 scripts/ground_truth_recorder.py --output-dir results/manual_01

# Publisher 터미널
python3 scripts/trajectory_publisher.py \
  --csv trajectories/human_1_right_turn.csv \
  --experiment-dir results/manual_01 \
  --wait-for-subscribers --ros-args -p use_sim_time:=true

# 재생 후 Recorder를 Ctrl+C로 종료하고 계산
python3 scripts/trajectory_validator.py --experiment-dir results/manual_01
```

`trajectory_validator.py`는 `validation_results.csv`, `aligned_samples.csv`, `validation_metadata.json`만 생성합니다. 이미지 생성 기능은 삭제했습니다. 이 오프라인 경로는 원본 GT 샘플 시각에서 지표를 계산하고 기본 최대 관측 간격 0.5초를 검사합니다. UI처럼 빈 구간을 채우는 비교는 `live_comparison.csv`와 `live_metrics.csv`를 사용하세요.

## 구현 파일

- `scripts/trajectory_viewer.py`: PyQt5 실시간 창, ROS 수신, CSV 선택·재생, 원본 GT 및 비교 CSV 저장.
- `scripts/trajectory_live_data.py`: 시간 정렬, GT 선형 보간, yaw 경계 처리, 화면 비교 지표.
- `scripts/ground_truth_recorder.py`: 실제 관측 기록 기능 재사용.
- `scripts/trajectory_publisher.py`: 기존 재생·메타데이터 기능 재사용.
- `scripts/trajectory_validator.py`: PNG 저장을 제거한 CSV 지표 계산.

기존 C++ Controller와 World는 이번 UI 변경으로 수정하지 않습니다. Python 3, ROS 2 Humble, PyQt5, Matplotlib이 필요하며 현재 환경에 설치되어 있습니다. 경로를 직접 그리는 편집 기능은 이후 Task 3-5에서 같은 UI에 추가했습니다.

## 검증과 보관 결과

실제 Gazebo 렌더링과 분리된 ROS 도메인에서 UI가 실제 GT를 받고 재생·보간·CSV 저장·전체 시간 범위 비교를 수행하는 것을 확인했습니다. 테스트 소스 파일은 저장소에 추가하지 않았습니다.

이전 기록의 원본 CSV와 수치 지표는 [실험 보고서](../results/task34_real_02/REPORT.md)에 보존합니다. 경로·오차 PNG 14개는 사용자 요청으로 삭제했습니다. 오래된 캐시 수집 방식의 진단 실험 `results/task34_real_01`은 INVALIDATED 상태를 유지합니다.

전체 Jackal·LiDAR·Teleop 성능 회귀 검사와 세 Actor 동시 실행은 이번 UI 변경에서 검증하지 않았습니다.
