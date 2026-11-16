"""PX4 v1.16 상태를 구독하는 단일 ROS 2 노드."""

import math
import time
from dataclasses import dataclass
from typing import Any

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
    VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import config


@dataclass
class ReceivedState:
    """최신 메시지와 로컬 수신/데이터 갱신 시각을 저장한다."""

    message: Any = None
    received_monotonic_s: float | None = None
    advanced_monotonic_s: float | None = None

    def update(self, message: Any, now_s: float) -> None:
        # timestamp는 PX4 메시지의 us 값이다. 로컬 단조 시계와 빼지 않는다.
        if self.message is None or message.timestamp != self.message.timestamp:
            self.advanced_monotonic_s = now_s
        self.message = message
        self.received_monotonic_s = now_s

    def problem(self, now_s: float) -> str | None:
        if self.message is None:
            return "미수신"
        if now_s - self.received_monotonic_s > config.DATA_TIMEOUT_S:
            return "수신 중단"
        if self.message.timestamp == 0:
            return "timestamp=0"
        if now_s - self.advanced_monotonic_s > config.DATA_TIMEOUT_S:
            return "timestamp 갱신 중단"
        return None


class PX4Interface(Node):
    """PX4 통신을 담당한다. 비행 단계의 결정은 mission.py에서 한다."""

    def __init__(self) -> None:
        super().__init__(config.NODE_NAME)
        self.local_position = ReceivedState()
        self.vehicle_status = ReceivedState()
        self.land_detected = ReceivedState()
        self._flight_publishers = None

        # PX4의 BEST_EFFORT publisher와 호환. VOLATILE은 새 데이터를 받는다.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=config.QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions_px4 = [
            self.create_subscription(
                VehicleLocalPosition, config.LOCAL_POSITION_TOPIC,
                self._on_local_position, qos,
            ),
            self.create_subscription(
                VehicleStatus, config.VEHICLE_STATUS_TOPIC,
                self._on_vehicle_status, qos,
            ),
            self.create_subscription(
                VehicleLandDetected, config.LAND_DETECTED_TOPIC,
                self._on_land_detected, qos,
            ),
        ]
        # 숫자를 직접 나열하지 않고 설치된 메시지의 모드 상수를 사용한다.
        prefix = "NAVIGATION_STATE_"
        self._nav_names = {
            getattr(VehicleStatus, name): name[len(prefix):]
            for name in dir(VehicleStatus)
            if name.startswith(prefix) and name != prefix + "MAX"
        }
        self._print_timer = self.create_timer(config.PRINT_PERIOD_S, self.print_state)
        self.get_logger().info(
            "PX4 상태 모니터 시작 | "
            f"토픽: {config.LOCAL_POSITION_TOPIC}, "
            f"{config.VEHICLE_STATUS_TOPIC}, {config.LAND_DETECTED_TOPIC}"
        )

    def _on_local_position(self, message: VehicleLocalPosition) -> None:
        self.local_position.update(message, time.monotonic())

    def _on_vehicle_status(self, message: VehicleStatus) -> None:
        self.vehicle_status.update(message, time.monotonic())

    def _on_land_detected(self, message: VehicleLandDetected) -> None:
        self.land_detected.update(message, time.monotonic())

    def enable_flight_publishers(self) -> None:
        """main에서 SITL과 중복 제어 확인을 끝낸 뒤 한 번 호출한다."""
        self._flight_publishers = (
            self.create_publisher(OffboardControlMode, config.OFFBOARD_TOPIC, 10),
            self.create_publisher(TrajectorySetpoint, config.TRAJECTORY_TOPIC, 10),
            self.create_publisher(VehicleCommand, config.COMMAND_TOPIC, 10),
        )
        self._ack_subscription = self.create_subscription(
            VehicleCommandAck, config.COMMAND_ACK_TOPIC, self._on_ack,
            QoSProfile(depth=config.QOS_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def _on_ack(self, message: VehicleCommandAck) -> None:
        if (message.target_system, message.target_component) == (
            config.SOURCE_SYSTEM, config.SOURCE_COMPONENT,
        ):
            self.get_logger().info(
                f"Command ACK: command={message.command}, result={message.result} "
                "(0=accepted, 1=temporary reject, 2=denied, 4=failed)"
            )

    def publish_position_ned(self, position_ned, yaw_ned_rad: float) -> None:
        timestamp_us = self.get_clock().now().nanoseconds // 1000
        self._flight_publishers[0].publish(
            OffboardControlMode(timestamp=timestamp_us, position=True)
        )
        nan = float("nan")
        self._flight_publishers[1].publish(TrajectorySetpoint(
            timestamp=timestamp_us, position=list(position_ned), yaw=yaw_ned_rad,
            velocity=[nan] * 3, acceleration=[nan] * 3, jerk=[nan] * 3, yawspeed=nan,
        ))

    def send_command(self, name: str) -> None:
        status = self.vehicle_status.message
        command = VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            target_system=status.system_id, target_component=status.component_id,
            source_system=config.SOURCE_SYSTEM, source_component=config.SOURCE_COMPONENT,
            from_external=True,
        )
        if name == "OFFBOARD":
            command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            command.param1, command.param2 = 1.0, 6.0
        elif name == "ARM":
            command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            command.param1 = 1.0
        elif name == "LAND":
            command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
            # NaN은 현재 위치에서 착륙한다는 의미. 위도/경도 0을 보내지 않는다.
            command.param4 = command.param5 = command.param6 = command.param7 = float("nan")
        else:
            raise ValueError(f"지원하지 않는 명령: {name}")
        self._flight_publishers[2].publish(command)
        self.get_logger().info(f"Command 전송: {name}")

    def state_problems(self, now_s: float) -> list[str]:
        """수신 상태와 위치/방향의 유효성을 검사한다. 비행 허가 판정은 아니다."""
        problems = []
        for name, state in (
            ("위치", self.local_position),
            ("기체 상태", self.vehicle_status),
            ("착륙 상태", self.land_detected),
        ):
            problem = state.problem(now_s)
            if problem:
                problems.append(f"{name}: {problem}")
        position = self.local_position.message
        if position is not None:
            if not position.xy_valid or not position.z_valid:
                problems.append("위치 추정 무효")
            if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
                problems.append("위치에 NaN/Inf")
            if not math.isfinite(position.heading):
                problems.append("heading에 NaN/Inf")
            elif not position.heading_good_for_control:
                # v1.16의 자력계 사용 시 지상에서는 최종 정렬 전일 수 있다.
                problems.append("heading 최종 정렬 미완료")
        status = self.vehicle_status.message
        if status is not None:
            if status.failsafe:
                problems.append("PX4 failsafe 활성")
            if not status.pre_flight_checks_pass:
                problems.append("PX4 preflight 미통과")
        return problems

    def print_state(self) -> None:
        now_s = time.monotonic()
        problems = self.state_problems(now_s)
        lines = ["상태: " + ("; ".join(problems) if problems else "수신·추정 유효")]
        position = self.local_position.message
        if position is not None:
            lines.append(
                f"  NED 위치 [m]: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}"
                f" | heading={position.heading:.3f} rad ({math.degrees(position.heading):.1f}°)"
            )
            lines.append(
                f"  xy_valid={position.xy_valid}, z_valid={position.z_valid}, "
                f"heading_good_for_control={position.heading_good_for_control}"
                f" | EKF reset counters xy/z/heading="
                f"{position.xy_reset_counter}/{position.z_reset_counter}/{position.heading_reset_counter}"
            )
        status = self.vehicle_status.message
        if status is not None:
            arm_name = {
                VehicleStatus.ARMING_STATE_DISARMED: "DISARMED",
                VehicleStatus.ARMING_STATE_ARMED: "ARMED",
            }.get(status.arming_state, "UNKNOWN")
            lines.append(
                f"  Arm={arm_name} ({status.arming_state})"
                f" | mode={self._nav_names.get(status.nav_state, 'UNKNOWN')} ({status.nav_state})"
                f" | failsafe={status.failsafe}, preflight={status.pre_flight_checks_pass}"
            )
        land = self.land_detected.message
        if land is not None:
            lines.append(f"  landed={land.landed}, ground_contact={land.ground_contact}")
        # 마지막 값이 남아 있어도 오래된 데이터이면 위 상태 행에 표시한다.
        for name, state in (
            ("위치", self.local_position), ("상태", self.vehicle_status),
            ("착륙", self.land_detected),
        ):
            if state.message is not None:
                lines.append(
                    f"  {name}: timestamp_us={state.message.timestamp}, "
                    f"수신 후 경과={now_s - state.received_monotonic_s:.2f}s"
                )
        self.get_logger().info("\n".join(lines))
