"""STEP 3/4/6 상태 머신. ROS 의존성 없는 NED 계산 모듈."""

from dataclasses import dataclass
import math

import config as c
from frame_transform import MissionFrame, capture_relative_target_ned, wrap_angle
from trajectory import CircleTrajectory, OrbitProgress, inward_yaw_ned, line_position, smooth_fraction


@dataclass(frozen=True)
class Telemetry:
    fresh: bool
    status_fresh: bool
    position_valid: bool
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    yaw_ned_rad: float
    armed: bool
    landed: bool
    nav_state: int
    preflight: bool
    failsafe: bool
    resets: tuple[int, int, int]
    delta_heading_rad: float


@dataclass
class Control:
    position_ned: tuple[float, float, float] | None = None
    yaw_ned_rad: float = 0.0
    command: str | None = None


class FlightMission:
    """명령을 보냈다는 사실 대신 실제 상태를 확인하여 전환한다."""

    OFFBOARD = 14
    AUTO_LAND = 18
    TERMINAL = {"DONE", "FAILED"}

    def __init__(self, now_s: float, frame_test: bool = False,
                 orbit: bool = False, fixed_yaw: bool = False):
        self.state = "WAIT"
        self.entered_s = now_s
        self.last_tick_s = now_s
        self.stable_since_s = None
        self.last_command_s = -math.inf
        self.origin_ned = None
        self.target_ned = None
        self.yaw_ned_rad = 0.0
        self.resets = None
        self.failure = ""
        self.ever_armed = False
        self.land_handed_over = False
        self.max_altitude_m = 0.0
        self.max_horizontal_m = 0.0
        self.prepare_started_s = None
        self.warmup_restarts = 0
        self.frame_test = frame_test
        self.frame = None
        self.segment_start_ned = None
        self.segment_goal_ned = None
        self.segment_start_yaw_rad = None
        self.segment_goal_yaw_rad = None
        self.forward_start_ned = None
        self.forward_heading_ned_rad = None
        self.orbit_mode = orbit or fixed_yaw
        self.fixed_yaw = fixed_yaw
        if frame_test and self.orbit_mode:
            raise ValueError("좌표 변환 시험과 원주비행 모드는 동시에 사용할 수 없습니다.")
        turns = c.FIXED_YAW_TEST_TURNS if fixed_yaw else c.ORBIT_TURNS
        self.circle = CircleTrajectory(c.ORBIT_RADIUS_M, c.ORBIT_SPEED_M_S, turns,
                                       c.TAKEOFF_ALTITUDE_M, c.ORBIT_CLOCKWISE,
                                       c.ORBIT_SPEED_RAMP_S) if self.orbit_mode else None
        self.horizontal_limit_m = (c.ORBIT_RADIUS_M+c.ORBIT_POSITION_MARGIN_M
                                   if self.orbit_mode else c.MAX_HORIZONTAL_DISTANCE_M)
        self.orbit_progress = None
        self.target_angle_rad = 0.0
        self.actual_angle_rad = 0.0
        self.radius_m = math.nan
        self.radial_error_m = math.nan
        self.orbit_error_since_s = None
        self.inward_start_yaw_rad = None
        self.inward_goal_yaw_rad = None
        self.fixed_orbit_yaw_rad = None

    @property
    def finished(self):
        return self.state in self.TERMINAL

    def transition(self, state: str, now_s: float):
        self.state = state
        self.entered_s = now_s
        self.stable_since_s = None
        self.last_command_s = -math.inf
        if state == "PREPARE_OFFBOARD":
            self.prepare_started_s = now_s

    def stable_for(self, condition: bool, duration_s: float, now_s: float) -> bool:
        if not condition:
            self.stable_since_s = None
        elif self.stable_since_s is None:
            self.stable_since_s = now_s
        return self.stable_since_s is not None and now_s - self.stable_since_s >= duration_s

    def abort(self, reason: str, t: Telemetry, now_s: float):
        if not self.failure:
            self.failure = reason
        if self.finished or self.state == "LAND":
            return
        if t.status_fresh and not t.armed and not self.ever_armed:
            self.transition("FAILED", now_s)
        else:
            if t.fresh and t.position_valid:
                self.target_ned = t.position_ned
            self.transition("LAND", now_s)

    def tick(self, now_s: float, t: Telemetry) -> Control:
        out = Control()
        if self.finished:
            return out
        gap_s = now_s - self.last_tick_s
        self.last_tick_s = now_s
        if self.origin_ned is not None and t.fresh and t.position_valid:
            altitude = self.origin_ned[2] - t.position_ned[2]
            horizontal = math.hypot(
                t.position_ned[0] - self.origin_ned[0],
                t.position_ned[1] - self.origin_ned[1],
            )
            self.max_altitude_m = max(self.max_altitude_m, altitude)
            self.max_horizontal_m = max(self.max_horizontal_m, horizontal)
            if self.orbit_mode:
                self.radius_m = horizontal
                self.radial_error_m = horizontal-c.ORBIT_RADIUS_M
        self.ever_armed |= t.status_fresh and t.armed and self.state != "WAIT"

        if self.state == "WAIT":
            if t.status_fresh and t.armed:
                self.failure = "시작 시 이미 Armed인 기체입니다."
                self.transition("FAILED", now_s)
                return out
            # 재실행 시 Disarmed/Offboard에 남으면 heartbeat 부재 자체로
            # preflight=False일 수 있다. 지상 준비 스트림만 먼저 허용한다.
            ready = (t.fresh and t.position_valid and t.landed
                     and (t.preflight or t.nav_state == self.OFFBOARD)
                     and not t.failsafe and math.isfinite(t.yaw_ned_rad))
            if self.stable_for(ready, c.INITIAL_STABLE_S, now_s):
                self.origin_ned = t.position_ned
                self.frame = MissionFrame(t.position_ned, t.yaw_ned_rad)
                self.target_ned = t.position_ned
                self.yaw_ned_rad = t.yaw_ned_rad
                self.resets = t.resets
                self.transition("PREPARE_OFFBOARD", now_s)
            elif now_s - self.entered_s > c.WAIT_TIMEOUT_S:
                self.failure = "유효한 지상 초기 상태 수신 시간 초과"
                self.transition("FAILED", now_s)
            return out

        if self.state != "LAND":
            # 지상·Arm 전의 준비 스트림은 끊기면 처음부터 다시 채울 수 있다.
            # 모드 명령 이후와 비행 중의 지연 중단 조건은 그대로 적용한다.
            can_restart_warmup = (
                self.state == "PREPARE_OFFBOARD" and t.fresh and t.position_valid
                and not t.armed and not self.ever_armed and t.landed
                and (t.preflight or t.nav_state == self.OFFBOARD) and not t.failsafe
            )
            if gap_s > c.CONTROL_GAP_LIMIT_S and can_restart_warmup:
                self.entered_s = now_s
                self.warmup_restarts += 1
            if not t.fresh or not t.position_valid:
                self.abort("데이터 수신 중단 또는 위치/속도 무효", t, now_s)
            elif gap_s > c.CONTROL_GAP_LIMIT_S and not can_restart_warmup:
                self.abort("제어 주기 지연", t, now_s)
            elif t.failsafe:
                self.failure = "PX4 failsafe가 제어권을 인수했습니다."
                self.transition("FAILED", now_s)
                return out
            elif t.resets[:2] != self.resets[:2]:
                self.abort("EKF 위치 좌표 재설정", t, now_s)
            elif horizontal > self.horizontal_limit_m or altitude > c.MAX_ALTITUDE_M:
                self.abort("허용 비행 범위 초과", t, now_s)
            elif self.ever_armed and not t.armed:
                self.abort("미션 도중 예상하지 못한 Disarm", t, now_s)
            elif self.state in ("TAKEOFF", "HOVER", "YAW_TEST", "FORWARD_TEST", "RETURN_TEST",
                                "FORWARD", "TURN_INWARD", "ORBIT", "ORBIT_SETTLE", "RETURN") and t.nav_state != self.OFFBOARD:
                if t.nav_state == self.AUTO_LAND:
                    self.abort("PX4가 착륙 모드로 전환됨", t, now_s)
                else:
                    self.failure = "Offboard 상실: 외부/사용자 모드에 제어권을 넘깁니다."
                    self.transition("FAILED", now_s)
                    return out
            if self.state != "LAND" and t.resets[2] != self.resets[2]:
                if self.orbit_mode and self.state in ("FORWARD", "TURN_INWARD", "ORBIT", "ORBIT_SETTLE", "RETURN"):
                    self.abort("수평 미션 도중 EKF heading 재설정", t, now_s)
                else:
                    # 수직 준비 중 yaw 기준이 보정되면 초기 방향 추정도 보정한다.
                    self.yaw_ned_rad = wrap_angle(self.yaw_ned_rad + t.delta_heading_rad)
                    if self.orbit_mode:
                        self.frame = MissionFrame(self.origin_ned,
                            self.frame.initial_heading_ned_rad + t.delta_heading_rad)
                    if self.state == "YAW_TEST":
                        self.segment_start_yaw_rad = wrap_angle(self.segment_start_yaw_rad + t.delta_heading_rad)
                        self.segment_goal_yaw_rad = wrap_angle(self.segment_goal_yaw_rad + t.delta_heading_rad)
                    self.resets = t.resets

        elapsed_s = now_s - self.entered_s
        if self.state == "PREPARE_OFFBOARD":
            yaw_error = math.atan2(math.sin(t.yaw_ned_rad - self.yaw_ned_rad),
                                   math.cos(t.yaw_ned_rad - self.yaw_ned_rad))
            if now_s - self.prepare_started_s > c.PREPARE_TIMEOUT_S:
                self.abort("Offboard 준비 스트림 안정화 시간 초과", t, now_s)
            elif abs(yaw_error) > c.MAX_GROUND_YAW_DRIFT_RAD:
                self.abort("준비 중 초기 heading 변동 과다", t, now_s)
            elif elapsed_s >= c.PRESTREAM_S:
                self.transition("OFFBOARD", now_s)
        elif self.state == "OFFBOARD":
            if t.nav_state == self.OFFBOARD:
                self.transition("ARM", now_s)
            else:
                out.command = self.retry("OFFBOARD", now_s)
                if elapsed_s > c.MODE_TIMEOUT_S:
                    self.abort("Offboard 전환 시간 초과", t, now_s)
        elif self.state == "ARM":
            if t.nav_state != self.OFFBOARD:
                self.abort("Arm 이전 Offboard 상실", t, now_s)
            elif t.armed:
                self.transition("TAKEOFF", now_s)
            elif not t.preflight:
                self.abort("Arm 이전 PX4 preflight 미통과", t, now_s)
            elif elapsed_s > c.MODE_TIMEOUT_S:
                self.abort("Arm 시간 초과", t, now_s)
            else:
                out.command = self.retry("ARM", now_s)
        elif self.state in ("TAKEOFF", "HOVER"):
            goal = self.frame.mission_to_ned((0.0, 0.0, -c.TAKEOFF_ALTITUDE_M))
            if self.state == "TAKEOFF":
                fraction = smooth_fraction(elapsed_s, c.TAKEOFF_RAMP_S)
                self.target_ned = (*goal[:2], self.origin_ned[2] - c.TAKEOFF_ALTITUDE_M*fraction)
            else:
                self.target_ned = goal
            if math.dist(self.target_ned, t.position_ned) > c.MAX_TRACKING_ERROR_M:
                self.abort("목표 추종 오차 초과", t, now_s)
            else:
                settled = (math.dist(goal, t.position_ned) <= c.POSITION_TOLERANCE_M
                           and math.sqrt(sum(v*v for v in t.velocity_ned)) <= c.SPEED_TOLERANCE_M_S
                           and not t.landed)
                if self.state == "TAKEOFF":
                    if self.stable_for(elapsed_s >= c.TAKEOFF_RAMP_S and settled, c.SETTLE_S, now_s):
                        self.transition("HOVER", now_s)
                    elif elapsed_s > c.TAKEOFF_TIMEOUT_S:
                        self.abort("이륙 시간 초과", t, now_s)
                elif self.stable_for(settled, c.HOVER_S, now_s):
                    if self.orbit_mode:
                        self.segment_start_ned = self.target_ned
                        self.segment_goal_ned = self.frame.mission_to_ned(
                            (c.ORBIT_RADIUS_M, 0.0, -c.TAKEOFF_ALTITUDE_M))
                        self.yaw_ned_rad = self.frame.mission_yaw_to_ned(0.0)
                        self.transition("FORWARD", now_s)
                    elif self.frame_test:
                        self.segment_start_yaw_rad = self.yaw_ned_rad
                        self.segment_goal_yaw_rad = wrap_angle(self.yaw_ned_rad + c.FRAME_TEST_YAW_RAD)
                        self.transition("YAW_TEST", now_s)
                    else:
                        self.transition("LAND", now_s)
                elif elapsed_s > c.TAKEOFF_TIMEOUT_S:
                    self.abort("호버링 안정화 시간 초과", t, now_s)

        elif self.state in ("FORWARD", "RETURN"):
            duration = c.ORBIT_FORWARD_S if self.state == "FORWARD" else c.ORBIT_RETURN_S
            self.target_ned = line_position(self.segment_start_ned, self.segment_goal_ned,
                                            elapsed_s, duration)
            if self.state == "RETURN":
                self.yaw_ned_rad = inward_yaw_ned(t.position_ned, self.origin_ned,
                    self.yaw_ned_rad, c.CENTER_YAW_HOLD_RADIUS_M)
            settled = self.at_goal(t, self.segment_goal_ned, self.yaw_ned_rad)
            if math.dist(self.target_ned, t.position_ned) > c.MAX_TRACKING_ERROR_M:
                self.abort("직선 이동 추종 오차 초과", t, now_s)
            elif self.stable_for(elapsed_s >= duration and settled, c.SETTLE_S, now_s):
                if self.state == "FORWARD":
                    self.inward_start_yaw_rad = t.yaw_ned_rad
                    self.inward_goal_yaw_rad = inward_yaw_ned(t.position_ned, self.origin_ned,
                        self.yaw_ned_rad, c.CENTER_YAW_HOLD_RADIUS_M)
                    self.transition("TURN_INWARD", now_s)
                else:
                    self.transition("LAND", now_s)
            elif elapsed_s > duration+c.ORBIT_STAGE_TIMEOUT_S:
                self.abort("직선 이동 안정화 시간 초과", t, now_s)

        elif self.state == "TURN_INWARD":
            fraction = smooth_fraction(elapsed_s, c.ORBIT_TURN_S)
            # 약 180도 회전의 부호가 작은 추정 오차로 바뀌지 않도록 양의 방향 선택.
            delta = (self.inward_goal_yaw_rad-self.inward_start_yaw_rad) % (2*math.pi)
            self.yaw_ned_rad = wrap_angle(self.inward_start_yaw_rad + fraction*delta)
            if elapsed_s >= c.ORBIT_TURN_S:
                self.yaw_ned_rad = inward_yaw_ned(t.position_ned, self.origin_ned,
                    self.yaw_ned_rad, c.CENTER_YAW_HOLD_RADIUS_M)
            settled = self.at_goal(t, self.segment_goal_ned, self.yaw_ned_rad)
            if self.stable_for(elapsed_s >= c.ORBIT_TURN_S and settled, c.SETTLE_S, now_s):
                self.fixed_orbit_yaw_rad = self.yaw_ned_rad
                self.orbit_progress = OrbitProgress(c.ORBIT_MIN_VALID_RADIUS_M, c.ORBIT_MAX_ANGLE_STEP_RAD)
                self.orbit_progress.update(self.frame.ned_to_mission(t.position_ned))
                self.transition("ORBIT", now_s)
            elif elapsed_s > c.ORBIT_TURN_S+c.ORBIT_STAGE_TIMEOUT_S:
                self.abort("중심 지향 회전 시간 초과", t, now_s)

        elif self.state in ("ORBIT", "ORBIT_SETTLE"):
            sample = self.circle.sample(elapsed_s if self.state == "ORBIT" else self.circle.duration_s)
            self.target_ned = self.frame.mission_to_ned(sample.position_mission)
            self.target_angle_rad = sample.angle_rad
            self.yaw_ned_rad = (self.fixed_orbit_yaw_rad if self.fixed_yaw else
                inward_yaw_ned(t.position_ned, self.origin_ned, self.yaw_ned_rad,
                               c.CENTER_YAW_HOLD_RADIUS_M))
            position_mission = self.frame.ned_to_mission(t.position_ned)
            self.radius_m = math.hypot(*position_mission[:2])
            self.radial_error_m = self.radius_m-c.ORBIT_RADIUS_M
            try:
                self.actual_angle_rad = self.orbit_progress.update(position_mission)
            except ValueError as error:
                self.abort(str(error), t, now_s)
            if self.state != "LAND":
                tracking_bad = (
                    abs(self.radial_error_m) > c.ORBIT_MAX_RADIAL_ERROR_M
                    or math.dist(self.target_ned, t.position_ned) > c.MAX_TRACKING_ERROR_M
                    or abs(wrap_angle(t.yaw_ned_rad-self.yaw_ned_rad)) > c.ORBIT_MAX_YAW_ERROR_RAD
                )
                if not tracking_bad:
                    self.orbit_error_since_s = None
                elif self.orbit_error_since_s is None:
                    self.orbit_error_since_s = now_s
                if self.orbit_error_since_s is not None and now_s-self.orbit_error_since_s >= c.ORBIT_ERROR_HOLD_S:
                    self.abort("원주비행 반경/위치/yaw 추종 오차 지속", t, now_s)
                else:
                    actual_complete = (self.circle.direction*self.actual_angle_rad >=
                        2*math.pi*self.circle.turns-c.ORBIT_ANGLE_TOLERANCE_RAD)
                    settled = actual_complete and self.at_goal(t, self.target_ned, self.yaw_ned_rad)
                    if self.state == "ORBIT":
                        if sample.finished and settled:
                            self.transition("ORBIT_SETTLE", now_s)
                        elif elapsed_s > self.circle.duration_s+c.ORBIT_COMPLETION_TIMEOUT_S:
                            self.abort("목표/실제 원주비행 완료 확인 시간 초과", t, now_s)
                    elif self.stable_for(settled, c.SETTLE_S, now_s):
                        self.segment_start_ned = self.target_ned
                        self.segment_goal_ned = self.frame.mission_to_ned((0.0, 0.0, -c.TAKEOFF_ALTITUDE_M))
                        self.transition("RETURN", now_s)
                    elif elapsed_s > c.ORBIT_STAGE_TIMEOUT_S:
                        self.abort("원주비행 종료 안정화 시간 초과", t, now_s)

        elif self.state in ("YAW_TEST", "FORWARD_TEST", "RETURN_TEST"):
            yaw_phase = self.state == "YAW_TEST"
            ramp_s = c.FRAME_TEST_YAW_RAMP_S if yaw_phase else c.FRAME_TEST_MOVE_S
            fraction = smooth_fraction(elapsed_s, ramp_s)
            if yaw_phase:
                yaw_delta = wrap_angle(self.segment_goal_yaw_rad - self.segment_start_yaw_rad)
                self.yaw_ned_rad = wrap_angle(self.segment_start_yaw_rad + fraction*yaw_delta)
                goal_yaw = self.segment_goal_yaw_rad
            else:
                self.target_ned = tuple(
                    start + fraction*(goal-start)
                    for start, goal in zip(self.segment_start_ned, self.segment_goal_ned)
                )
                goal_yaw = self.yaw_ned_rad
            position_goal = self.target_ned if yaw_phase else self.segment_goal_ned
            settled = (
                elapsed_s >= ramp_s
                and math.dist(position_goal, t.position_ned) <= c.POSITION_TOLERANCE_M
                and abs(wrap_angle(t.yaw_ned_rad - goal_yaw)) <= c.YAW_TOLERANCE_RAD
                and math.sqrt(sum(v*v for v in t.velocity_ned)) <= c.SPEED_TOLERANCE_M_S
                and not t.landed
            )
            if math.dist(self.target_ned, t.position_ned) > c.MAX_TRACKING_ERROR_M:
                self.abort("좌표 변환 시험 추종 오차 초과", t, now_s)
            elif self.stable_for(settled, c.SETTLE_S, now_s):
                if self.state == "YAW_TEST":
                    # 실제 위치/heading을 이 순간에 한 번만 캡처한다.
                    self.forward_start_ned = self.segment_start_ned = t.position_ned
                    self.forward_heading_ned_rad = t.yaw_ned_rad
                    self.segment_goal_ned = capture_relative_target_ned(
                        t.position_ned, t.yaw_ned_rad, (c.FRAME_TEST_FORWARD_M, 0.0, 0.0),
                    )
                    self.transition("FORWARD_TEST", now_s)
                elif self.state == "FORWARD_TEST":
                    self.segment_start_ned = t.position_ned
                    self.segment_goal_ned = self.frame.mission_to_ned((0.0, 0.0, -c.TAKEOFF_ALTITUDE_M))
                    self.transition("RETURN_TEST", now_s)
                else:
                    self.transition("LAND", now_s)
            elif elapsed_s > c.FRAME_TEST_TIMEOUT_S:
                self.abort("좌표 변환 시험 시간 초과", t, now_s)

        if self.state == "LAND":
            if t.fresh and t.landed and not t.armed:
                self.transition("DONE", now_s)
                return Control()
            if t.status_fresh and t.nav_state == self.AUTO_LAND:
                self.land_handed_over = True
            if now_s - self.entered_s > c.LAND_TIMEOUT_S:
                self.failure = self.failure or "착륙/Disarm 확인 시간 초과: QGC에서 상태 확인 필요"
                self.transition("FAILED", now_s)
                return Control()
            if t.status_fresh and t.armed and not self.land_handed_over:
                out.command = self.retry("LAND", now_s)
            if self.land_handed_over:
                # PX4 착륙 모드 인수 후 setpoint/heartbeat를 발행하지 않는다.
                return out

        if not self.finished and t.fresh and t.position_valid:
            out.position_ned = self.target_ned
            out.yaw_ned_rad = self.yaw_ned_rad
        if self.finished:
            return Control()
        return out

    def at_goal(self, t: Telemetry, position_ned, yaw_ned_rad) -> bool:
        return (math.dist(position_ned, t.position_ned) <= c.POSITION_TOLERANCE_M
                and abs(wrap_angle(t.yaw_ned_rad-yaw_ned_rad)) <= c.YAW_TOLERANCE_RAD
                and math.sqrt(sum(v*v for v in t.velocity_ned)) <= c.SPEED_TOLERANCE_M_S
                and not t.landed)

    def retry(self, command: str, now_s: float) -> str | None:
        if now_s - self.last_command_s >= c.COMMAND_RETRY_S:
            self.last_command_s = now_s
            return command
        return None
