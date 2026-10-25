# Task 3-4 실제 Gazebo 검증 결과

2026-10-25 (KST). Gazebo Fortress의 실제 렌더링 SceneUpdate에서 Actor Visual의 WorldPose와 Scene.Time을 관측했습니다. 목표 명령을 GT로 복사하지 않았습니다.

| 경로 | 비교 샘플 | Mean (m) | RMSE (m) | Max (m) | Mean absolute yaw (rad) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 직선 | 81 | 1.87644e-08 | 2.53163e-08 | 4.76837e-08 | 0 |
| 우회전 | 81 | 2.90616e-08 | 3.55132e-08 | 8.1874e-08 | 1.54252e-08 |
| 사각형 | 161 | 9.32942e-09 | 1.26484e-08 | 2.38419e-08 | 1.11712e-16 |

모든 경로에서 GT가 Target 전체 시간 범위를 포함했습니다. 시간 오프셋을 최적화하지 않았으며 Publisher가 저장한 재생 원점을 사용했습니다. 현재 Controller가 pose를 직접 배치하는 방식이고 기본 비교는 이전 샘플 유지 방식이므로 오차가 수치 정밀도 수준입니다. 실제 사람·로봇의 물리적 추종 정확도를 의미하지 않습니다.

## 관측 조건

연속 경로 기준 `linear`로 같은 우회전을 별도 평가한 결과는 [validation_linear](turn/validation_linear/validation_results.csv)에 저장했습니다. Mean 0.03568 m, RMSE 0.04086 m, Max 0.05576 m, Mean Absolute Yaw Error 0.01784 rad입니다. 샘플 유지 제어와 연속 목표 경로의 차이를 포함하는 평가입니다.

- `walking_person` 한 명 / 논리 ID `human_1`.
- 실제 원본 World를 복사한 시험 World에 64×64 카메라를 추가하여 서버 렌더링 활성화. 원본 World는 수정하지 않음.
- EGL headless 소프트웨어 렌더링. 카메라 데이터의 Ignition 구독 활성화.
- GT 최대 20 Hz, Target 10 Hz. `world` 좌표, 같은 `/clock` 시간축.
- pose/info 및 dynamic_pose/info의 실제 메시지에 Actor가 없음을 확인: [pose/info](pose_info.txt), [dynamic_pose/info](dynamic_pose_info.txt).

## 파일

- 직선: [GT CSV](straight/ground_truth.csv), [Target CSV](straight/target.csv), [지표](straight/validation/validation_results.csv).
- 우회전: [GT CSV](turn/ground_truth.csv), [Target CSV](turn/target.csv), [지표](turn/validation/validation_results.csv).
- 사각형: [GT CSV](square/ground_truth.csv), [Target CSV](square/target.csv), [지표](square/validation/validation_results.csv).

## 한계와 미검증 항목

- 이 실험은 실제 렌더링 데이터를 사용했지만 GUI 화면을 직접 관찰하는 검사는 수행하지 않았습니다.
- Jackal을 함께 구동한 전체 센서·Teleop 회귀 검사는 수행하지 않았습니다.
- 세 Actor 동시 실행은 검증하지 않았습니다. ID 매핑·보행자별 파일 처리 구조는 준비했습니다.
- 일부 Publisher 실행에서 ACK 대기 시간 초과 경고가 남았습니다. GT 시간 범위와 마지막 목표 관측은 별도로 확인했습니다.
- `../task34_real_01`은 초기 캐시 관측 방식의 문제를 드러낸 진단 실험입니다. 오래된 pose에 새 시각이 붙어 해당 실험은 INVALIDATED로 표시했으며 정상 GT/성능 결과로 사용하지 않습니다.

경로·오차 PNG 기록은 실시간 UI로 전환하면서 사용자 요청으로 삭제했습니다. 원본 CSV와 수치 지표는 보존합니다.
