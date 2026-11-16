"""상태 확인, 수직/좌표 변환 시험, --orbit 원주비행 프로그램."""

import argparse
import csv
from datetime import datetime
import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

import config as c
from mission import FlightMission, Telemetry
from frame_transform import heading_frd_displacement_to_ned, wrap_angle
from trajectory import inward_yaw_ned
from px4_interface import PX4Interface
from sitl_check import verify_local_sitl


def telemetry(node: PX4Interface, now_s: float) -> Telemetry:
    """ROS 메시지를 순수 Python 상태 머신의 입력으로 바꾼다."""
    p = node.local_position.message
    s = node.vehicle_status.message
    land = node.land_detected.message
    states = (node.local_position, node.vehicle_status, node.land_detected)
    fresh = all(
        state.problem(now_s) is None
        and now_s - state.received_monotonic_s <= c.FLIGHT_DATA_TIMEOUT_S
        and now_s - state.advanced_monotonic_s <= c.FLIGHT_DATA_TIMEOUT_S
        for state in states
    )
    status_fresh = (
        node.vehicle_status.problem(now_s) is None
        and now_s - node.vehicle_status.received_monotonic_s <= c.FLIGHT_DATA_TIMEOUT_S
        and now_s - node.vehicle_status.advanced_monotonic_s <= c.FLIGHT_DATA_TIMEOUT_S
    )
    position_ned = (p.x, p.y, p.z) if p else (math.nan,) * 3
    velocity_ned = (p.vx, p.vy, p.vz) if p else (math.nan,) * 3
    valid = bool(p and p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid
                 and all(math.isfinite(v) for v in (*position_ned, *velocity_ned, p.heading)))
    return Telemetry(
        fresh, status_fresh, valid, position_ned, velocity_ned,
        p.heading if p else math.nan,
        bool(s and s.arming_state == s.ARMING_STATE_ARMED),
        bool(land and land.landed), s.nav_state if s else -1,
        bool(s and s.pre_flight_checks_pass), bool(s and s.failsafe),
        (p.xy_reset_counter, p.z_reset_counter, p.heading_reset_counter) if p else (0, 0, 0),
        p.delta_heading if p else 0.0,
    )


def check_flight_graph(node: PX4Interface, expected_system: int) -> None:
    # 발견 시간을 주되 이 동안에는 비행 publisher를 만들지 않는다.
    discovery_until_s = time.monotonic() + 3.0
    while time.monotonic() < discovery_until_s:
        rclpy.spin_once(node, timeout_sec=0.1)
    status = node.vehicle_status.message
    if status is None or status.system_id != expected_system:
        raise RuntimeError("로컬 SITL의 system_id와 수신 기체가 일치하지 않습니다.")
    for topic in (c.LOCAL_POSITION_TOPIC, c.VEHICLE_STATUS_TOPIC, c.LAND_DETECTED_TOPIC):
        if node.count_publishers(topic) != 1:
            raise RuntimeError(f"기체 상태 publisher가 한 개가 아닙니다: {topic}")
    for topic in (c.OFFBOARD_TOPIC, c.TRAJECTORY_TOPIC, c.COMMAND_TOPIC):
        if node.count_publishers(topic) != 0 or node.count_subscribers(topic) != 1:
            raise RuntimeError(f"중복 제어 또는 PX4 subscriber 미확인: {topic}")


class FlightSession:
    """단일 ROS 노드에 미션 타이머와 검증 CSV를 연결한다."""

    def __init__(self, node: PX4Interface, frame_test: bool = False,
                 orbit: bool = False, fixed_yaw: bool = False):
        self.node = node
        self.started_s = time.monotonic()
        self.mission = FlightMission(self.started_s, frame_test=frame_test,
                                      orbit=orbit, fixed_yaw=fixed_yaw)
        c.LOG_DIR.mkdir(parents=True, exist_ok=True)
        stage = ("step6_fixed_yaw" if fixed_yaw else "step6") if orbit else ("step4" if frame_test else "step3")
        self.log_path = c.LOG_DIR / f"{stage}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.csv"
        self.log_file = self.log_path.open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.log_file)
        self.writer.writerow((
            "elapsed_s", "state", "x_ned_m", "y_ned_m", "z_ned_m",
            "target_x_ned_m", "target_y_ned_m", "target_z_ned_m",
            "yaw_ned_rad", "target_yaw_ned_rad", "armed", "landed",
            "nav_state", "fresh", "failure", "control_gap_s", "publish_duration_s",
            "x_mission_m", "y_mission_m", "z_mission_m", "vx_ned_m_s", "vy_ned_m_s", "vz_ned_m_s",
            "forward_start_x_ned_m", "forward_start_y_ned_m", "forward_heading_ned_rad",
            "target_x_mission_m", "target_y_mission_m", "target_z_mission_m",
            "radius_m", "radial_error_m", "target_angle_rad", "actual_angle_rad",
            "yaw_error_rad", "inward_yaw_error_rad", "vx_heading_frd_m_s", "vy_heading_frd_m_s",
        ))
        self.abort_requested = ""
        self.last_flush_s = self.started_s
        node.enable_flight_publishers()
        self.timer = node.create_timer(c.CONTROL_PERIOD_S, self.step)
        node.get_logger().info(f"{stage.upper()} 미션 시작 | CSV: {self.log_path}")

    def step(self):
        now_s = time.monotonic()
        state = telemetry(self.node, now_s)
        previous = self.mission.state
        previous_restarts = self.mission.warmup_restarts
        control_gap_s = now_s - self.mission.last_tick_s
        if self.abort_requested:
            self.mission.abort(self.abort_requested, state, now_s)
        output = self.mission.tick(now_s, state)
        publish_started_s = time.monotonic()
        if output.position_ned is not None:
            self.node.publish_position_ned(output.position_ned, output.yaw_ned_rad)
        if output.command:
            self.node.send_command(output.command)
        publish_duration_s = time.monotonic() - publish_started_s
        if self.mission.warmup_restarts != previous_restarts:
            self.node.get_logger().warning(
                f"Arm 전 준비 지연 {control_gap_s:.3f}s: "
                f"Offboard 연속 준비 {c.PRESTREAM_S:.1f}초를 다시 시작합니다."
            )
        if previous != self.mission.state:
            self.node.get_logger().info(
                f"미션: {previous} → {self.mission.state} | {self.mission.failure}"
            )
        target = self.mission.target_ned or (math.nan,) * 3
        position_mission = (math.nan,) * 3
        target_mission = (math.nan,) * 3
        inward_error_rad = math.nan
        velocity_heading_frd = (math.nan,) * 3
        if self.mission.frame is not None and state.position_valid:
            position_mission = self.mission.frame.ned_to_mission(state.position_ned)
            target_mission = self.mission.frame.ned_to_mission(target)
            inward_yaw = inward_yaw_ned(state.position_ned, self.mission.origin_ned,
                                       self.mission.yaw_ned_rad, c.CENTER_YAW_HOLD_RADIUS_M)
            inward_error_rad = wrap_angle(state.yaw_ned_rad-inward_yaw)
            velocity_heading_frd = heading_frd_displacement_to_ned(state.velocity_ned, -state.yaw_ned_rad)
        forward_start = self.mission.forward_start_ned or (math.nan,) * 3
        self.writer.writerow((
            f"{now_s-self.started_s:.3f}", self.mission.state, *state.position_ned,
            *target, state.yaw_ned_rad, self.mission.yaw_ned_rad, state.armed,
            state.landed, state.nav_state, state.fresh, self.mission.failure,
            f"{control_gap_s:.6f}", f"{publish_duration_s:.6f}",
            *position_mission, *state.velocity_ned, *forward_start[:2],
            self.mission.forward_heading_ned_rad,
            *target_mission, self.mission.radius_m, self.mission.radial_error_m,
            self.mission.target_angle_rad, self.mission.actual_angle_rad,
            wrap_angle(state.yaw_ned_rad-self.mission.yaw_ned_rad) if state.position_valid else math.nan,
            inward_error_rad, *velocity_heading_frd[:2],
        ))
        if now_s - self.last_flush_s >= 1.0:
            self.log_file.flush()
            self.last_flush_s = now_s
            if self.mission.state == "ORBIT":
                m = self.mission
                self.node.get_logger().info(
                    f"ORBIT: 목표={m.circle.direction*m.target_angle_rad/(2*math.pi):.3f}바퀴, "
                    f"실제={m.circle.direction*m.actual_angle_rad/(2*math.pi):.3f}바퀴, "
                    f"반경 오차={m.radial_error_m:.3f}m, "
                    f"yaw 오차={math.degrees(wrap_angle(state.yaw_ned_rad-m.yaw_ned_rad)):.1f}°"
                )
        if self.mission.finished:
            self.timer.cancel()

    def close(self):
        self.timer.cancel()
        self.log_file.close()
        m = self.mission
        self.node.get_logger().info(
            f"결과: {m.state}, 최대 상대 고도={m.max_altitude_m:.2f}m, "
            f"최대 수평 이탈={m.max_horizontal_m:.2f}m | {m.failure or '정상 완료'}"
        )


def positive_seconds(value: str) -> float:
    seconds = float(value)
    if not 0 < seconds < float("inf"):
        raise argparse.ArgumentTypeError("실행 시간은 유한한 양수여야 합니다.")
    return seconds


def main() -> int:
    parser = argparse.ArgumentParser(description="PX4 상태 확인 / STEP 3·4·6 SITL 비행")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--takeoff", action="store_true", help="4m 이륙 → 호버링 → 착륙")
    modes.add_argument("--frame-test", action="store_true", help="4m 이륙 → yaw 90° → 노즈 방향 1m 이동 → 복귀 → 착륙")
    modes.add_argument("--orbit", action="store_true", help="4m 이륙 → 전진 → 중심 지향 원주비행 두 바퀴 → 복귀·착륙")
    modes.add_argument("--orbit-fixed-yaw", action="store_true", help="고정 yaw로 한 바퀴 원 경로 추종 검증")
    parser.add_argument("--duration", type=positive_seconds, metavar="SECONDS",
                        help="상태 확인 모드의 실행 시간")
    options, ros_args = parser.parse_known_args()
    flight_requested = options.takeoff or options.frame_test or options.orbit or options.orbit_fixed_yaw
    if flight_requested and options.duration is not None:
        parser.error("--duration은 상태 확인 모드에서만 사용합니다.")
    expected_system = verify_local_sitl() if flight_requested else None
    # SIGINT에서 ROS context를 즉시 닫지 않고 착륙 상태 머신을 계속 실행한다.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    session = None
    stop_requested = False

    def on_signal(signum, _frame):
        nonlocal stop_requested
        stop_requested = True
        if session is not None:
            session.abort_requested = f"사용자 중단 (signal={signum})"

    previous_handlers = {sig: signal.signal(sig, on_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = PX4Interface()
        if flight_requested:
            check_flight_graph(node, expected_system)
            if stop_requested:
                return 1
            session = FlightSession(node, frame_test=options.frame_test,
                                    orbit=options.orbit or options.orbit_fixed_yaw,
                                    fixed_yaw=options.orbit_fixed_yaw)
        started_s = time.monotonic()
        while rclpy.ok():
            if session:
                if session.mission.finished:
                    return 0 if session.mission.state == "DONE" and not session.mission.failure else 1
            elif stop_requested or (
                options.duration is not None and time.monotonic() - started_s >= options.duration
            ):
                break
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                raise
            except Exception as error:
                if session is None:
                    raise
                node.get_logger().error(f"실행 오류, 착륙 요청: {error}")
                session.abort_requested = f"실행 오류: {error}"
        return 0
    except ExternalShutdownException:
        return 1
    finally:
        if session is not None:
            session.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
