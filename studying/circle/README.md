# PX4 + ROS 2 원주비행 — Baylands 실행 안내

Gazebo의 `baylands` 월드에서 기본 `x500`을 실행하는 방법이다.
터미널 1 → 2 → 3 → 4 순서로 진행한다.

## 기존 시뮬레이션에서 월드를 바꾸는 경우

1. 비행 중이면 착륙과 Disarmed까지 기다린다. Python 비행 중 `Ctrl+C`는 착륙 요청으로 처리된다.
2. 터미널 3에서 `Ctrl+C`로 기존 PX4 SITL을 종료하고 기존 Gazebo도 종료한다.
   기존 Gazebo 서버가 남아 있으면 새 월드 대신 기존 월드에 연결될 수 있다.
3. QGroundControl과 XRCE Agent가 이미 실행 중이면 터미널 1·2는 그대로 사용한다.
   같은 프로그램을 중복 실행하지 않는다.

## 터미널 1 — QGroundControl

실행 위치: `~/Downloads`

```bash
cd ~/Downloads
./QGroundControl-x86_64.AppImage
```

QGroundControl을 켜 둔다. 터미널 3에서 PX4가 기동하면 연결 상태를 확인한다.

## 터미널 2 — Micro XRCE-DDS Agent

실행 위치는 어느 폴더든 가능하다. 이 컴퓨터의 Agent는 `ros2_px4_ws` 환경을 source해야 한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_px4_ws/install/setup.bash
export ROS_DOMAIN_ID=0

MicroXRCEAgent udp4 -p 8888
```

Agent는 실행 상태로 유지한다.

## 터미널 3 — PX4 SITL + Gazebo Baylands

실행 위치: `~/PX4-Autopilot`

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

unset PX4_UXRCE_DDS_NS PX4_GZ_MODEL_NAME PX4_GZ_STANDALONE
export PX4_UXRCE_DDS_PORT=8888

cd ~/PX4-Autopilot
PX4_GZ_WORLD=baylands PX4_SIM_SPEED_FACTOR=1 make px4_sitl gz_x500
```

`PX4_GZ_WORLD=baylands`가 회색 기본 월드 대신 Baylands 배경을 선택한다.
처음 실행하면 외부 지형 모델을 다운로드하므로 인터넷 연결이 필요하고 로딩이 길어질 수 있다.
Gazebo에 지형과 드론이 나타나고 QGroundControl에 기체가 연결될 때까지 기다린다.

## 터미널 4 — 상태 확인 및 원주비행

실행 위치: `~/space_y/studying`

먼저 환경을 설정하고 3초간 상태를 확인한다. 이 명령은 비행 명령을 보내지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_px4_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR=~/space_y/studying/logs/ros

cd ~/space_y/studying
python3 main.py --duration 3
```

확인할 항목:

- `xy_valid=True`, `z_valid=True`
- `Arm=DISARMED`, `landed=True`
- 수신 중단 없이 timestamp가 갱신됨

지상에서는 `heading 최종 정렬 미완료`가 표시될 수 있다.
이전 비행 후 Disarmed/Offboard 상태에서는 준비 heartbeat 전까지 preflight가 false일 수 있다.
프로그램은 실제 Arm 직전에 PX4 preflight 통과를 확인한다.

기존에 승인한 착륙 감속 설정이 유지됐는지 확인한다.

```bash
python3 landing_profile.py
```

현재 값과 목표가 다르면, Disarmed 상태에서 기존 프로파일을 재적용할 수 있다.
이 명령은 원래 값을 `logs/`에 백업하고 PX4 파라미터를 변경한다.

```bash
python3 landing_profile.py --apply
```

준비가 끝나면 아래 명령으로 자동 비행을 시작한다. **이 명령은 SITL 드론을 Arm하고 이륙시킨다.**

```bash
python3 main.py --orbit
```

비행 순서:

```text
4m 이륙 → 초기 노즈 방향으로 4m 전진 → 중심을 향해 약 180° 회전
→ 반경 4m의 원을 시계 방향으로 두 바퀴 비행
→ 원 시작점에서 안정화 → 이륙 지점으로 복귀 → 착륙 → Disarm 확인 → DONE
```

현재 원주 속도는 0.75m/s이며, 전체 미션은 기본 월드 검증에서 약 2분 34초 걸렸다.
QGroundControl의 이륙 슬라이더를 따로 조작할 필요는 없다.
정상 완료 시 터미널에 `결과: DONE`이 표시되고 기체가 착륙·Disarmed 상태가 된다.

## Baylands 사용 시 확인할 점

- Baylands 월드 파일은 설치되어 있지만, 이 프로젝트의 실제 비행 검증은 현재까지
  평평한 `default` 월드에서 수행했다. Baylands에서의 비행 성공은 아직 검증하지 않았다.
- 현재 프로그램에는 장애물 회피나 지형 추종 기능이 없다.
  이륙 지점 주변 지름 8m의 원 경로와 고도 4m 공간에 장애물이 없는지 확인한다.
- 고도 4m는 **이륙 지점 기준 상대 고도**다. 경로 아래 지형이 높아져도 자동으로 상승하지 않는다.
- 월드 변경 후 오류가 발생하면 다음 단계로 진행하지 않고 터미널 3·4 로그를 확인한다.

상세 개발 과정과 검증 결과는 [process.md](process.md)에 정리되어 있다.
월드 선택 방식: [PX4 v1.16 Gazebo 문서](https://docs.px4.io/v1.16/en/sim_gazebo_gz/#specify-world).
