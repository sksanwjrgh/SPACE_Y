"""고정 Mission-FRD와 NED, heading 기준 수평 FRD 변환. m, rad 단위.

heading 기준 FRD는 yaw만 사용한다. roll/pitch를 포함하는 Body-FRD와 다르다.
"""

from dataclasses import dataclass
import math

Vector3 = tuple[float, float, float]


def _vector3(value) -> Vector3:
    result = tuple(float(v) for v in value)
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError("좌표는 유한한 값 세 개여야 합니다.")
    return result


def wrap_angle(angle_rad: float) -> float:
    """각도를 [-pi, pi)로 정규화한다. +pi와 -pi는 같은 방향이다."""
    if not math.isfinite(angle_rad):
        raise ValueError("각도는 유한해야 합니다.")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def heading_frd_displacement_to_ned(displacement_heading_frd, heading_ned_rad: float) -> Vector3:
    """현재 heading 기준의 수평 전후/좌우/수직 이동량을 NED로 회전한다."""
    x, y, z = _vector3(displacement_heading_frd)
    heading = wrap_angle(heading_ned_rad)
    cosine, sine = math.cos(heading), math.sin(heading)
    return cosine*x - sine*y, sine*x + cosine*y, z


def capture_relative_target_ned(start_position_ned, start_heading_ned_rad: float,
                                displacement_heading_frd) -> Vector3:
    """명령 시작 순간에 한 번 계산하여 고정할 목표 위치를 반환한다."""
    start = _vector3(start_position_ned)
    delta = heading_frd_displacement_to_ned(displacement_heading_frd, start_heading_ned_rad)
    return tuple(p + d for p, d in zip(start, delta))


@dataclass(frozen=True)
class MissionFrame:
    """이륙 위치와 초기 heading에 고정한 내부 비행 계획 좌표계."""

    origin_ned: Vector3
    initial_heading_ned_rad: float

    def __post_init__(self):
        object.__setattr__(self, "origin_ned", _vector3(self.origin_ned))
        object.__setattr__(self, "initial_heading_ned_rad", wrap_angle(self.initial_heading_ned_rad))

    def mission_to_ned(self, position_mission) -> Vector3:
        return capture_relative_target_ned(
            self.origin_ned, self.initial_heading_ned_rad, position_mission,
        )

    def ned_to_mission(self, position_ned) -> Vector3:
        position = _vector3(position_ned)
        delta_ned = tuple(p-o for p, o in zip(position, self.origin_ned))
        return heading_frd_displacement_to_ned(delta_ned, -self.initial_heading_ned_rad)

    def mission_yaw_to_ned(self, yaw_mission_rad: float) -> float:
        return wrap_angle(self.initial_heading_ned_rad + yaw_mission_rad)

    def ned_yaw_to_mission(self, yaw_ned_rad: float) -> float:
        return wrap_angle(yaw_ned_rad - self.initial_heading_ned_rad)
