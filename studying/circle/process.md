STEP 1. 시뮬레이션 환경 확인

기존에 설치한 PX4 SITL, Gazebo, ROS2 Humble, Micro XRCE-DDS Agent를 이용해 기본 x500 드론을 스폰한다. QGroundControl에서 드론이 연결되는지 확인한다.

이번에는 LiDAR나 ArUco를 사용하지 않으므로 기존 자율착륙 launch 파일은 실행할 필요가 없어.



STEP 2. Python으로 드론 상태 받아오기

첫 번째로 px4_interface.py를 개발한다.

ROS2를 통해 드론의 현재 x, y, z 위치와 Yaw, Arm 상태를 받아온다. 아직 드론을 움직이지는 않는다.

목표: Python 프로그램이 드론의 현재 상태를 정상적으로 인식하는 것.



STEP 3. Offboard 제어 및 수직 이륙

main.py와 mission.py의 기본 구조를 작성한다.

Offboard 전환 → Arm → 고도 4m 이륙 → 호버링 → 착륙까지만 구현한다.

목표: Python만으로 드론을 이륙시키고 안전하게 착륙시키는 것.



STEP 4. FRD 좌표계 구현

frame_transform.py를 개발한다.

드론의 현재 방향을 읽어 전방, 후방, 좌우 이동 명령을 NED 목표값으로 변환한다.

특히 기체가 90° 회전한 후에도 전진 명령이 실제 노즈 방향으로 작동하는지 테스트한다.

목표: 기체 방향과 무관하게 FRD 이동 명령을 사용할 수 있는 것.



STEP 5. 직선 이동과 Yaw 제어

고도 4m까지 이륙한 후 앞으로 4m 이동한다. 이동을 완료하면 180° 회전해서 이륙 지점을 바라보도록 한다.

목표: 드론의 위치와 방향을 독립적으로 제어하는 것.



STEP 6. 원주비행 경로 생성

trajectory.py를 개발한다.

우선 Yaw를 고정하고 반지름 4m인 원을 한 바퀴 돌아본다. 그다음 현재 위치를 이용해 중심을 향하는 Yaw를 계속 계산하도록 추가한다.

목표: 원의 반지름을 유지하며 노즈가 중심을 바라보는 비행.



STEP 7. 전체 미션 통합 및 검증

두 바퀴 비행, 원점 복귀, 착륙을 하나의 미션으로 연결한다. 비행 기록을 저장하고 실제 궤적과 목표 궤적을 비교한다.

목표: 프로그램을 한 번 실행하면 전체 비행이 자동으로 완료되는 것.

---

현재 진행 결과: STEP 6 원주비행 구현 및 SITL 검증 완료

- STEP 1: Ubuntu 22.04.5, ROS 2 Humble, Python 3.10.12, PX4 v1.16.0,
  Gazebo Harmonic 8.15.0, Micro XRCE-DDS Agent 2.4.2 확인.
- 사용할 메시지 환경: ~/ros2_px4_ws (~/px4_ros_ws는 이 경로의 링크).
  px4_msgs release/1.16의 필요한 7개 메시지 정의가 PX4와 일치한다.
- STEP 2: 위치·기체·착륙 상태의 실시간 수신 검증 완료.
  VehicleStatus 토픽은 /fmu/out/vehicle_status_v1이다.
- STEP 3: 기본 x500 SITL에서 Offboard → Arm → 4m 이륙 → 5초 호버링
  → AUTO_LAND → landed=true 및 Disarmed 확인 → DONE까지 실행 성공.

현재 파일 역할

- config.py: 상태 수신과 STEP 3/4/6 비행 파라미터, 승인된 착륙 감속 프로파일.
- px4_interface.py: ROS 통신. 상태 확인 모드에서는 구독만 하며,
  비행 모드에서 heartbeat·위치 목표·명령 publisher와 ACK subscriber를 추가한다.
- mission.py: ROS 의존성 없는 FlightMission 상태 머신. 수직·좌표 시험·원주비행 선택.
- main.py: 실행 모드 선택, 단일 노드/미션 연결, SIGINT 처리, 검증 CSV 저장.
- sitl_check.py: 명령 발행 전 로컬 기본 x500 SITL과 자동 Disarm 설정을 조회한다.
  기존 PX4 설정을 변경하지 않는다.
- frame_transform.py: ROS 의존성 없는 Mission-FRD↔NED, heading 기준 수평 FRD 변환.
- trajectory.py: 부드러운 직선·원 경로, 실제 위치 기반 중심 지향 yaw, 실제 회전각 누적.
- flight_logger.py: 비행 CSV의 요약 JSON과 궤적·고도·오차 PNG 생성. ROS 연결 불필요.
- landing_profile.py: 착륙 파라미터 조회·적용·백업·복원. 기본 실행은 조회만 한다.
- tests/: 좌표 변환·수신·미션·착륙 프로파일·원 경로의 단위 테스트 56개.

실행 방법 (QGC, Agent, 기본 x500 SITL이 실행 중인 상태)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_px4_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR=~/space_y/studying/logs/ros
cd ~/space_y/studying

# 상태 확인: Ctrl+C로 종료
python3 main.py

# STEP 3: 준비부터 실제 SITL Arm·이륙·착륙까지 자동 수행
python3 main.py --takeoff
```

비행 중 Ctrl+C/SIGTERM은 착륙 요청으로 처리하고, 착륙·Disarm을 확인할 때까지
프로그램을 유지한다. 이 중단 흐름은 단위 테스트로 확인했으며 실제 SITL에서
사용자 중단을 발생시키는 비행 시험은 아직 수행하지 않았다.
--duration 옵션은 상태 확인 모드에서만 사용할 수 있다.

제어 방식

- 제어 주기는 20Hz, Offboard 준비 스트림은 2초.
- 초기 NED 위치를 (x0,y0,z0)로 저장하고 최종 목표는 (x0,y0,z0-4).
- 12초짜리 5차 보간으로 이륙 목표 고도를 부드럽게 올린다.
- 위치 오차 0.25m 이내, 속도 0.25m/s 이하 상태가 2초 유지되면 호버링으로 전환.
- 목표 위치에서 5초간 안정적으로 호버링한 뒤 PX4 착륙 명령을 요청한다.
- 실제 AUTO_LAND 확인 후 Offboard heartbeat와 위치 목표 발행을 중지한다.
- 강제 Arm과 공중 Disarm 명령을 사용하지 않는다.
- heading_good_for_control은 자력계 사용 시 비행 중 최종 정렬 완료까지 false일 수 있다.
  STEP 3에서는 유효한 위치/속도, 유한한 heading, PX4 preflight를 확인한다.
  비행 중 heading reset은 delta_heading만큼 목표 yaw를 보정하고,
  XY/Z 좌표 reset은 미션을 중단해 착륙 요청으로 전환한다.
- 데이터 단절 시 heartbeat를 중지하며 PX4의 기존 Offboard-loss 설정에 의존한다.
  현재 확인한 설정: COM_OF_LOSS_T=1초, COM_OBL_RC_ACT=0 (Position).
  이 설정은 자동 착륙을 의미하지 않는다. 실제 통신 단절 비행 시험은 미수행.

실행 검증 결과

- CSV: logs/step3_20261117_005942_944232.csv
- 샘플 658개, 미션 시간 32.902초 (시작 전 DDS 발견 시간 제외).
- 최대 상대 고도: 4.011m.
- 호버링 고도 최솟값/평균/최댓값: 3.965 / 3.991 / 4.010m.
- 최대 수평 이탈: 0.081m (저장한 초기 NED 위치 기준).
- 제어 주기 평균/최대: 0.050 / 0.051초 (CSV 시간 반올림 기준).
- 최종 상태: DONE, armed=false, landed=true, fresh=true, failure 없음.
- Offboard·Arm·Land 명령 ACK는 모두 accepted.
- 비행 중 heading reset counter가 1에서 2로 바뀌었고 최종 정렬 true 확인.
- 단위 테스트 20개 통과. 통신 단절·사용자 모드 변경·좌표 reset 등의
  비정상 조건은 단위 테스트에서 확인했으며 실제 비행에서 모두 재현한 것은 아니다.

테스트 명령

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_px4_ws/install/setup.bash
cd ~/space_y/studying
python3 -m unittest discover -s tests -v
```

STEP 4 구현과 검증

frame_transform.py의 MissionFrame은 초기 위치 origin_ned와 초기 heading을 고정한다.
mission_to_ned()는 미션 목표를 PX4 좌표로 바꾸고, ned_to_mission()은 실제 위치를
미션 좌표로 되돌린다. yaw 변환과 wrap_angle()의 단위는 rad이며, 각도 범위는 [-pi, pi)다.

heading_frd_displacement_to_ned()는 현재 heading에 정렬된 이동량을 회전한다.
roll/pitch를 반영하는 전체 3차원 Body-FRD 변환은 아니다.
capture_relative_target_ned()로 이동 시작 시점의 실제 위치·heading을 한 번 저장하여
목표를 고정한다. 매 주기마다 현재 위치에 이동량을 다시 더하지 않는다.

예: 원점이 (10,20,-4)일 때 heading 0°에서 전진 1m는 (11,20,-4),
heading 90°에서 전진 1m는 (10,21,-4)다. Mission-FRD의 고정 축은
기체가 회전해도 그대로이며, 현재 노즈 기준 이동은 현재 heading을 사용한다.

검증 비행은 STEP 5 전체 미션과 구분하여 작은 1m 이동만 수행한다.

```bash
# 기존 QGC·Agent·기본 x500 SITL을 켠 상태, 터미널 4
source /opt/ros/humble/setup.bash
source ~/ros2_px4_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR=~/space_y/studying/logs/ros
cd ~/space_y/studying
python3 main.py --frame-test
```

4m 이륙 → 호버링 → yaw +90° → 실제 노즈 방향 1m 전진 → 원위치 복귀
→ AUTO_LAND → 착륙 및 Disarm 확인 순서다. QGC 이륙 슬라이더 조작은 필요 없다.

- CSV: logs/step4_20261117_012441_654437.csv
- 최종 상태: DONE, armed=false, landed=true, failure 없음.
- yaw 실제 변화: 89.975°.
- 이동 시작 heading 기준 실제 이동: 전방 1.022m, 우측 0.034m.
- 착륙 전 원위치 복귀 수평 오차: 0.049m.
- 최대 제어 간격: 0.051606초. 미션 시간: 61.102초.
- 요약: logs/step4_validation.json

재실행 보완

이전 비행 후 Disarmed/Offboard에 남으면 heartbeat 부재로 PX4 preflight가 false가
될 수 있다. 이 경우 지상의 유효한 상태에서 준비 스트림만 먼저 시작한다.
실제 Arm 직전에는 여전히 PX4 preflight 통과가 필수다.
Arm 전 준비 지연 시 2초 연속 스트림을 다시 채우고, 15초 안에 안정되지 않으면
중단한다. 모드 전환 명령 이후와 비행 중의 지연 중단 조건은 유지한다.

착륙 감속 (사용자 승인 후 적용)

기본 x500에 유효한 거리 센서가 없으므로 MPC_LAND_ALT3/CRWL은 적용되지 않는다.
현재 PX4 v1.16의 MPC_LAND_SPEED 명시 최솟값 0.6m/s 범위에서 조정했다.
AUTO_LAND를 그대로 사용하며 PX4의 고도 기반 감속을 이용한다.
평평한 기본 월드에서 지상고는 거리 센서 대신 home 고도/로컬 위치 추정에 의존한다.

| 파라미터 | 원래 값 | 승인된 값 |
|---|---:|---:|
| MPC_LAND_ALT1 | 10m | 1.5m |
| MPC_LAND_ALT2 | 5m | 1m |
| MPC_LAND_SPEED | 0.7m/s | 0.6m/s |
| MPC_Z_VEL_MAX_DN | 1.5m/s | 0.7m/s |

1.5m 위 목표 하강 속도는 최대 0.7m/s, 1m 이하 목표는 0.6m/s이며 중간 구간에서
보간한다. 실제 속도에는 제어기와 추정 오차가 있다. Z_VEL_MAX_DN은 착륙 이외의
수직 제어에도 적용되는 하강 속도 상한이다.
PX4가 설정을 자동 저장할 수 있다. 원래 값 백업:
logs/landing_backup_20261117_012631_394464.json

```bash
# 현재 값과 config.py의 프로파일 비교 (조회만)
python3 landing_profile.py
# 설정을 config.py에서 바꾼 뒤 명시적으로 적용할 때 (Disarmed/SITL 확인 포함)
python3 landing_profile.py --apply
# 원래 값으로 복원할 때
python3 landing_profile.py --restore logs/landing_backup_20261117_012631_394464.json
```

사용자 요청에 따라 STEP 5의 단독 테스트 비행은 생략하고 STEP 6으로 진행한다.
4m 전진과 중심 지향 회전 기능은 원주비행 전후의 기존 비행 절차에 포함한다.

착륙 감속 적용 후 SITL 검증 결과

- 사용자 승인 후 네 파라미터를 적용하고 각각 읽어 적용값을 확인했다.
- CSV: logs/step3_20261117_012643_693186.csv
- 요약: logs/landing_validation.json
- 이륙·호버링·착륙 미션 33.252초, 최종 DONE/Disarmed/landed=true.
- 초기 위치 기준 상대 고도 1.7~3.0m 구간의 실제 평균 하강 속도: 0.691m/s.
- 상대 고도 0.2~0.8m 구간의 실제 평균 하강 속도: 0.613m/s.
- AUTO_LAND 인수 후에는 프로그램의 heartbeat/위치 목표를 발행하지 않았다.
- 시험 종료 후 네 파라미터가 승인된 값으로 유지됨을 재조회했다.
- 설정은 현재 PX4에 적용되어 있다. config.py의 LANDING_PROFILE을 편집할 경우
  main.py가 자동 적용하는 것이 아니라 landing_profile.py --apply를 따로 실행한다.

STEP 6 원주비행 구현

기존 비행 절차는 유지한다.

```text
WAIT → PREPARE_OFFBOARD → OFFBOARD → ARM → TAKEOFF → HOVER
→ FORWARD → TURN_INWARD → ORBIT → ORBIT_SETTLE → RETURN → LAND → DONE
```

4m 수직 이륙 후 초기 노즈 방향으로 4m 전진한다. 반경 4m 시작점에서 정지해
원의 중심을 향하도록 약 180° 회전한다. 위에서 본 시계 방향으로 원주비행하며
원 시작점에 복귀해 안정화한 뒤, 고도 4m를 유지한 채 이륙 지점으로 돌아와 착륙한다.
착륙·Disarm을 확인한 뒤 종료하며 1m 이하 감속 설정은 그대로 사용한다.

실행 모드 (터미널 1~3의 QGC·Agent·기본 x500 SITL은 기존과 동일)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_px4_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR=~/space_y/studying/logs/ros
cd ~/space_y/studying

# 먼저 고정 yaw로 원 위치 추종 한 바퀴 검증
python3 main.py --orbit-fixed-yaw

# 실제 위치에서 중심을 바라보며 두 바퀴 비행
python3 main.py --orbit
```

두 명령을 동시에 실행하지 않는다. 각 명령이 착륙·Disarm 후 종료되면 다음 명령을
실행한다. QGC 이륙 슬라이더 조작은 필요 없다.
--takeoff, --frame-test와 상태 확인 모드도 계속 사용할 수 있다.

계산 모듈

- CircleTrajectory.sample(): Mission-FRD에서 x=R cos(theta), y=R sin(theta), z=-고도.
  원점은 이륙 지점이며 반경 4m, 현재 정속 목표 0.75m/s, 각속도 0.1875rad/s다.
- 원 경로의 시작/끝에 각각 3초 반코사인 속도 램프를 사용한다. 두 바퀴 목표 경로 시간은
  현재 속도에서는 67.02초 + 가감속 보정 3초 = 약 70.02초다.
  준비·전진·회전·복귀·착륙 시간은 별도다.
- line_position(): 5차 보간으로 전진/복귀의 양 끝 속도·가속도를 0으로 만든다.
- inward_yaw_ned(): 목표 위치가 아니라 실제 NED 위치에서 이륙 지점을 바라본다.
  복귀 중 중심으로부터 0.5m 이하에서는 마지막 유효 yaw를 유지한다.
- OrbitProgress.update(): atan2의 -pi/pi 경계를 풀어 실제 누적각을 계산한다.
  역방향 이동은 실제 회전 진행량을 감소시키며 단순히 절댓값으로 더하지 않는다.

주요 설정은 config.py의 ORBIT_* 항목이다. --orbit는 기본 두 바퀴 중심 지향,
--orbit-fixed-yaw는 별도 FIXED_YAW_TEST_TURNS=1 설정을 사용한다.
시계 방향의 NED/Mission-FRD 각도 증가 부호는 양수다.
CircleTrajectory가 계산하는 접선 속도는 분석과 이후 feedforward 확장을 위한 것이며,
현재 PX4에는 위치·yaw 목표를 발행한다. 실제 속도는 PX4 추종 성능에 따라 달라진다.

완료/중단 조건

- 목표 경로 완료와 실제 누적 회전각 완료를 모두 요구한다. 실제 회전각 허용 오차 5°,
  위치 오차 0.25m, yaw 오차 6°, 속도 0.25m/s 이하로 시작점 복귀를 확인한다.
- 원 시작점에서 추가 2초 안정화 후 중심으로 복귀한다.
- 원주비행 중 반경/위치/yaw 과도 오차가 2초 지속되면 착륙 요청으로 전환한다.
- 중심에 1m 이내로 접근하거나 실제 회전각이 한 주기에 30° 넘게 변하면 중단한다.
- 수평 원 미션 도중 EKF heading이 재설정되면 착륙 요청으로 전환한다.
  이륙 중 최종 정렬에 따른 yaw 보정은 전진 시작 전에 초기 방향 추정에 반영한다.
- 원주비행 모드의 수평 범위 제한은 반경+1.5m로 확장한다. 기존 수직/좌표 시험의
  1.5m 수평 범위 제한은 유지한다.
- 기존 데이터 신선도·Offboard 상실·단계별 시간 초과·사용자 중단 처리를 유지한다.

CSV에는 NED/Mission-FRD 실제·목표 위치, 실제 반경과 오차, 목표·실제 누적각,
목표 yaw 오차, 중심 지향 yaw 오차, heading 기준 전후/좌우 실제 속도를 기록한다.
실행 중 ORBIT 행에는 목표/실제 바퀴 수와 반경·yaw 오차가 1초마다 표시된다.
수평 heading 기준 속도는 roll/pitch를 포함한 전체 3D Body-FRD 속도와 다르다.

비행 후 분석

```bash
# 비행이 DONE으로 종료된 뒤, 출력에 표시된 실제 CSV 경로를 지정한다.
python3 flight_logger.py logs/step6_실제파일이름.csv --plot
```

같은 폴더에 *_summary.json과 *_report.png를 만든다. 그래프는 Mission-FRD 기준
위에서 본 궤적과 노즈 방향, 상대 고도, 반경 오차, yaw 오차를 표시한다.
반경 오차와 이동 중인 목표점에 대한 추종 오차는 별도로 계산한다.
위치 제어만 사용하는 현재 버전은 원의 반경을 유지하면서도 목표점을 시간상
뒤따를 수 있다. 목표와 실제 누적각을 구분하고, 종료 시에는 실제 기체의 복귀와
안정화를 기다린다. 단위 테스트의 추종 지연 모델도 이 차이를 반영한다.

고정 yaw 한 바퀴 SITL 검증 결과 (기존 속도 0.5m/s)

- CSV: logs/step6_fixed_yaw_20261117_013851_842029.csv
- 원주비행을 포함한 전체 미션: 136.902초.
- 최종 DONE, armed=false, landed=true, failure 없음.
- 실제 회전: 1.001731바퀴, 평균 반경: 3.993m.
- 최대 절대 반경 오차: 0.049m, 반경 오차 RMS: 0.018m.
- 최대 고정 yaw 오차: 0.085°.
- 이동 목표점에 대한 평균/최대 3D 추종 오차: 0.479/0.571m.
  원 경로의 반경 오차와 다른 값이며 대부분 시간상 추종 지연이다.
- 분석 JSON과 PNG는 같은 CSV 이름의 *_summary.json, *_report.png에 저장했다.

중심 지향 두 바퀴 SITL 검증 결과 (기존 속도 0.5m/s)

- CSV: logs/step6_20261117_014202_401737.csv
- 실제 누적 회전: 1.999934바퀴, 위에서 본 시계 방향.
- 평균 반경: 3.993m. 최대 절대 반경 오차: 0.051m, RMS: 0.018m.
- 원주비행 중 상대 고도 최솟값/평균/최댓값: 3.986/4.001/4.018m.
- 최대 중심 지향 yaw 오차: 3.292°, yaw 오차 RMS: 2.763°.
- 정속에 가까운 구간에서 heading 기준 실제 vx 평균 0.027m/s, vy 평균 -0.504m/s.
  기체의 노즈가 중심을 향하면서 왼쪽으로 이동하는 동작이 확인됐다.
- 이동 목표점에 대한 평균/최대 3D 추종 오차: 0.503/0.588m.
  위치만 제어하는 현재 구현의 시간상 추종 지연이며, 단계 완료의 0.25m 허용 오차와
  구분한다. 향후 속도 feedforward 추가 시 개선 여부를 비교할 수 있다.
- 최대 제어 간격: 0.051498초. 전체 미션: 187.201초 (약 3분 7초).
- 최종 DONE, armed=false, landed=true, failure 없음.
- 기존 1m 이하 착륙 감속 설정을 유지했다. 새 PX4 파라미터 변경은 하지 않았다.
- 수학·추종 지연 상태 머신·중단 조건 등 단위 테스트 56개 통과.
- 실제 SITL은 기본 x500/평평한 default 월드/시계 방향에서 검증했다.
  반시계 방향과 다양한 초기 heading은 단위 테스트로 검증했다.

```bash
# 이번 중심 지향 비행의 결과 분석 재실행
python3 flight_logger.py logs/step6_20261117_014202_401737.csv --plot
```

요약 JSON: logs/step6_20261117_014202_401737_summary.json
그래프 PNG: logs/step6_20261117_014202_401737_report.png

다음 STEP 7에서는 이 원주비행 프로그램의 전체 미션 완성도를 점검하고,
필요한 로그 분석과 추종 성능 개선을 진행한다. STEP 5 단독 비행은 생략한 상태다.

원주 속도 1.5배 변경 및 검증

- 사용자 요청으로 config.py의 ORBIT_SPEED_M_S를 0.5에서 0.75m/s로 변경했다.
- 반경·회전 횟수·방향·가감속 시간·비행 절차·착륙 설정은 유지했다.
- 각속도는 0.1875rad/s, 정속 한 바퀴는 약 33.51초.
- 가감속 포함 두 바퀴 목표 경로는 약 70.02초 (이전 약 103.53초).
- 단위 테스트 56개 통과 후 실제 SITL에서 --orbit를 실행했다.
- CSV: logs/step6_20261117_015358_262713.csv
- 최종 DONE/Disarmed/landed=true, 실제 회전 2.002541바퀴.
- 원주 구간 실제 평균 반경 3.997m, 최대 절대 반경 오차 0.050m.
- 최대 중심 지향 yaw 오차 4.607° (이전 0.5m/s 시험에서는 3.292°).
- 이동 목표점 평균/최대 추종 오차 0.731/0.847m로 증가했다.
  목표점의 시간상 추종 지연과 원의 반경 오차를 구분한다.
- 전체 미션 153.602초 (약 2분 34초), 원주 구간 약 70.1초.
  이륙·전진·회전·복귀·착륙 시간은 유지되어 전체 미션이 1.5배 빨라지는 것은 아니다.
- 요약/그래프: 같은 CSV 이름의 *_summary.json, *_report.png.
- 실행 명령은 기존과 동일: python3 main.py --orbit
