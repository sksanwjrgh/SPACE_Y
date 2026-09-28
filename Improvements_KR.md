# PX4 & ArUco 정밀 착륙 시스템 개선 보고서 (Precision Landing Improvements)

본 문서는 Gazebo Harmonic 시뮬레이션 및 ROS 2 Humble 환경에서 PX4 기반 드론의 ArUco 마커 정밀 착륙 성능을 향상시키기 위해 수행된 알고리즘 설계, 문제 진단, 제어기 튜닝 및 구현 사항을 정리한 보고서입니다.

---

## 1. 개요 (Overview)

기존 오픈소스 기반 제어 로직(`landing_test_vel.cpp`, `marker_recognition.py`)은 단순 방향 벡터 정규화 방식과 고정 픽셀 스케일링을 사용하여 다음과 같은 심각한 비행 불안정성을 유발했습니다:
- 고도 변화에 따른 극심한 제어 오차 (고고도 반응 지연, 저고도 과보정)
- 마커 이탈 시 고스트 추적 및 비정상 비행
- 정렬 전 무조건적인 급격한 하강
- 정렬 완료 후 수평 진자 진동 (Swing-Sway / Limit-Cycle Chattering)
- 방향 정렬 시 급격한 Yaw 스핀 회전

이를 해결하기 위해 **TDD (테스트 주도 개발)** 방식으로 비전 기하학 및 비행 제어 모듈을 재설계하고, 실제 비행 역학(Flight Dynamics)에 맞춘 안정화 작업을 완료했습니다.

---

## 2. 주요 개선 사항 상세 (Key Improvements)

### (1) 비전 기하학 기반 메트릭 역투영 (Metric Pinhole Back-Projection)
* **관련 파일**: [`vision_geometry.py`](file:///home/yoonseop/ws_KRAC/src/imagery_processing/imagery_processing/vision_geometry.py), [`marker_recognition.py`](file:///home/yoonseop/ws_KRAC/src/imagery_processing/imagery_processing/marker_recognition.py)
* **기존 문제**:
  - 오차 계산을 `dx / 500`과 같은 임의의 픽셀 나눗셈에 의존.
  - 고도 10m에서는 픽셀 이동 대비 실제 이동량이 커서 반응이 매우 둔감했고, 0.5m 저고도에서는 1픽셀 변화에도 급격하게 반응하여 드론이 요동침.
* **개선 내용**:
  - 카메라 캘리브레이션 내부 파라미터($f_x, f_y, c_x, c_y$)와 LiDAR 실시간 고도($Z$)를 결합한 정규 핀홀 카메라 역투영 모델 구축:
    $$X_{\text{FRD}} = \frac{(c_y - v) \cdot Z}{f_y}, \quad Y_{\text{FRD}} = \frac{(u - c_x) \cdot Z}{f_x}$$
  - 모든 수평 오차를 미터(m) 단위의 물리량으로 정밀하게 산출.
  - 고도 센서 이상치($Z \le 0.1\text{m}$ 또는 NaN) 방어 가드 조건 추가.

---

### (2) 타깃 상실 감시견 (Target Loss Watchdog Failsafe)
* **관련 파일**: [`vision_geometry.py`](file:///home/yoonseop/ws_KRAC/src/imagery_processing/imagery_processing/vision_geometry.py)
* **기존 문제**:
  - 하강 기류나 드론 틸팅으로 마커가 카메라 시야각(FOV)을 벗어날 경우, 변수(`self.x_m`, `self.y_m`)가 마지막 측정값을 그대로 유지(Ghost Tracking).
  - 마커가 사라졌음에도 마지막 방향으로 계속 가속 비행하는 위험 발생.
* **개선 내용**:
  - 타임아웃 디바운싱(0.3초) 워치독 로직 탑재.
  - 마커 미검출 시간이 0.3초를 초과하면 즉시 유효 좌표를 무효화(`NaN` 반환).
  - 제어기에서 `NaN` 수신 시 수평 속도를 즉시 0으로 락(lock)하고 제자리 호버링 유지.

---

### (3) 강하 수용 깔때기 (Descent Acceptance Funnel)
* **관련 파일**: [`precision_landing_controller.hpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/include/flight_control/precision_landing_controller.hpp)
* **기존 문제**:
  - 수평 위치 정렬 여부와 상관없이 무조건 0.5 m/s 속도로 수직 하강.
  - 바람이나 외란으로 정렬이 덜 된 상태에서도 지면에 충돌하듯 착륙하는 문제.
* **개선 내용**:
  - 고도에 비례하는 동적 원뿔형 깔때기 반경 정의:
    $$r_{\text{funnel}} = \max(r_{\text{min}}, \text{funnel\_ratio} \times Z) \quad (r_{\text{min}}=0.2\text{m}, \text{ratio}=0.25)$$
  - 수평 오차가 깔때기 반경 이내로 들어온 경우에만 하강 속도를 부드럽게 가속 ($0.05 \to 0.5\text{ m/s}$).
  - 오차가 크면 하강 속도를 최소 크립 속도($0.05\text{ m/s}$)로 억제하여 수평 정렬 우선 보장.

---

### (4) 수평 진자 진동(Swing-Sway) 및 리밋 사이클 제거
* **관련 파일**: [`precision_landing_controller.hpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/include/flight_control/precision_landing_controller.hpp), [`landing_test_vel.cpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/src/landing_test_vel.cpp)
* **현상**:
  - 마커 상공에 정렬한 후, 드론이 멈추지 않고 수평 축을 따라 시계추처럼 좌우로 흔들리는 진동 발생.
* **근본 원인 분석 및 해결**:
  1. **불연속 속도 바닥값 제거 (Limit Cycle)**:
     - 기존 코드에 `오차 > 15cm`일 때 최소 속도를 $0.3\text{ m/s}$로 강제 클램핑하는 로직이 있었음.
     - 이로 인해 15cm 경계선에서 속도가 계단식으로 급변하여 오버슈트 $\to$ 반대편 반동 $\to$ 무한 진동(채터링)을 유발.
     - $\to$ 바닥값을 제거하고 오차가 0에 가까워질수록 속도가 0으로 부드럽게 점근 수렴하도록 수정.
  2. **비디오 전송 지연 위상차(Phase Lag) 해소**:
     - GStreamer UDP 비디오 전송 및 디코딩에 약 80ms 지연 존재.
     - 현재 쿼터니언으로 영상 오차를 역회전(de-rotation)시킬 경우, "과거 영상"과 "현재 기체 기울기" 간의 시간차로 인해 제동 틸팅이 오히려 가속 명령으로 변환(양의 피드백 발생).
     - $\to$ 자세 역회전을 비활성화하고 순수 핀홀 투영 방식으로 전환하여 위상 지연 제거.
  3. **제어기 게인 최적화**:
     - 외란 PID 게인을 $K_p = 0.4, K_i = 0.01, K_d = 0.0$으로 설정.
     - 미분 댐핑($K_d$)을 외부에 무리하게 넣지 않고 PX4 내부의 고주파(250Hz) 속도 루프가 자체 관성을 완충하도록 위임.

---

### (5) 부드러운 반속(Half-Speed) Yaw 헤딩 회전 정렬
* **관련 파일**: [`precision_landing_controller.hpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/include/flight_control/precision_landing_controller.hpp), [`landing_test_vel.cpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/src/landing_test_vel.cpp)
* **기존 문제**:
  - `TrajectorySetpoint` 메시지의 `yaw` 기본값이 `0.0`으로 초기화되어 전달됨.
  - PX4가 이를 즉각적인 정북(0 rad) 스텝 명령으로 해석하여 기본 최대 자동 선회 속도($45^\circ/\text{s} \approx 0.785\text{ rad/s}$)로 급격히 회전함.
* **개선 내용**:
  - 스루레이트 제한기(Slew-Rate Limiter) `update_yaw(curr_yaw, dt)` 구현.
  - 최대 선회 각속도를 절반인 **$22.5^\circ/\text{s}$ ($0.3927\text{ rad/s}$)** 로 강제 제한.
  - 최단 회전 경로 계산($[-\pi, \pi]$ 정규화) 및 매 주기마다 계산된 `yaw`와 `yawspeed`를 PX4에 연속 스트리밍.
  - 정밀 착륙 완료(`FINISHED`) 모드에서도 마지막 헤딩을 유지하도록 보호.
  - ROS 2 파라미터(`target_yaw`, `max_yaw_rate`)로 외부 튜닝 가능하도록 노출.

---

### (6) 착륙 완료 구간(`FINISHED`) Failsafe 방지
* **관련 파일**: [`landing_test_vel.cpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/src/landing_test_vel.cpp)
* **기존 문제**:
  - 지면 근접(`low_enough`) 판단 후 `VEHICLE_CMD_NAV_LAND`를 전송하면서 Offboard 명령 전송이 끊기면 PX4가 통신 두절 페일세이프로 인식할 위험.
* **개선 내용**:
  - 기체의 실제 착륙 상태(`VehicleLandDetected.landed == true`)가 확인되어 시동이 꺼질 때까지 유효한 하강 setpoint를 지속적으로 퍼블리시하여 안정적인 접지 유도.

---

## 3. 검증 결과 (Verification & Test Results)

모든 알고리즘은 TDD 방법론에 따라 작성된 단위 테스트를 통과했습니다:

### 1) Python 비전 기하학 단위 테스트 (`pytest`)
* **테스트 대상**: [`test_vision_geometry.py`](file:///home/yoonseop/ws_KRAC/src/imagery_processing/test/test_vision_geometry.py)
* **결과**: **5/5 통과 (100%)**
  - 중심점 투영 검증
  - 고도 스케일링 선형성 검증
  - 저고도/무효 고도 가드 조건 검증
  - 롤/피치 자세 투영 검증
  - 0.3초 타깃 손실 워치독 타임아웃 검증

### 2) C++ 정밀 착륙 제어기 단위 테스트 (`colcon test` / GTest)
* **테스트 대상**: [`test_landing_controller.cpp`](file:///home/yoonseop/ws_KRAC/src/flight_control/test/test_landing_controller.cpp)
* **결과**: **7/7 통과 (100%)**
  - 비례 제어 응답 (`PID_ProportionalResponse`)
  - 속도 댐핑 응답 (`PID_VelocityDamping`)
  - 안티 와인드업 클램핑 (`PID_AntiWindup`)
  - 하강 깔때기 게이팅 (`DescentFunnel_Gating`)
  - 중심 근접 부드러운 감속 (`SmoothDeceleration_NearCenter`)
  - 타깃 상실 시 안전 호버링 (`TargetLoss_Hover`)
  - 반속 Yaw 회전 평활화 (`YawRate_HalfSpeedSmoothing`)

### 3) ROS 2 패키지 빌드
* `flight_control`, `imagery_processing`, `launch_package` 전체 컴파일 성공 (경고/에러 0건).

---

## 4. 파일 수정 내역 요약 (Modified Files)

| 파일 경로 | 수정 유형 | 주요 변경점 |
| :--- | :---: | :--- |
| `src/imagery_processing/imagery_processing/vision_geometry.py` | 신규 | 핀홀 역투영 기하학, 마커 손실 워치독 구현 |
| `src/imagery_processing/test/test_vision_geometry.py` | 신규 | 비전 기하학 5종 pytest 단위 테스트 |
| `src/imagery_processing/imagery_processing/marker_recognition.py` | 수정 | VisionGeometry 연동 및 안전성 강화 |
| `src/flight_control/include/flight_control/precision_landing_controller.hpp` | 신규 | 2D PID, 하강 깔때기, Yaw 스루레이트 제한기 구현 |
| `src/flight_control/test/test_landing_controller.cpp` | 신규 | 착륙 제어기 및 Yaw 제어 7종 GTest 단위 테스트 |
| `src/flight_control/src/landing_test_vel.cpp` | 수정 | 제어기 연동, 쿼터니언 Yaw 추출, FINISHED 유지 |
| `src/flight_control/CMakeLists.txt` | 수정 | 신규 GTest 타깃 및 헤더 의존성 등록 |
