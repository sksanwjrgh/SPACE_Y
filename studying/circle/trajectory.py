"""ROS에 의존하지 않는 경로 계산. 위치 m, 시간 s, 각도 rad 단위."""

from dataclasses import dataclass
import math

from frame_transform import wrap_angle


def smooth_fraction(elapsed_s: float, duration_s: float) -> float:
    """양 끝의 속도·가속도가 0인 5차 보간 비율."""
    if not math.isfinite(elapsed_s) or not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("시간은 유한하고 duration은 양수여야 합니다.")
    u = min(1.0, max(0.0, elapsed_s / duration_s))
    return 10*u**3 - 15*u**4 + 6*u**5


def line_position(start, goal, elapsed_s: float, duration_s: float):
    fraction = smooth_fraction(elapsed_s, duration_s)
    return tuple(a + fraction*(b-a) for a, b in zip(start, goal))


def inward_yaw_ned(position_ned, center_ned, last_yaw_rad: float,
                   hold_radius_m: float = 0.5) -> float:
    """실제 위치에서 중심을 바라본다. 중심 근처에서는 마지막 방향 유지."""
    if not all(math.isfinite(v) for v in (*position_ned, *center_ned, last_yaw_rad, hold_radius_m)):
        raise ValueError("위치와 방향은 유한해야 합니다.")
    if hold_radius_m < 0:
        raise ValueError("yaw 유지 반경은 음수일 수 없습니다.")
    dx, dy = center_ned[0]-position_ned[0], center_ned[1]-position_ned[1]
    if math.hypot(dx, dy) <= hold_radius_m:
        return wrap_angle(last_yaw_rad)
    return math.atan2(dy, dx)


@dataclass(frozen=True)
class OrbitSample:
    position_mission: tuple[float, float, float]
    velocity_mission: tuple[float, float, float]
    angle_rad: float
    speed_m_s: float
    finished: bool


@dataclass(frozen=True)
class CircleTrajectory:
    radius_m: float
    speed_m_s: float
    turns: int
    altitude_m: float
    clockwise: bool = True
    ramp_s: float = 3.0

    def __post_init__(self):
        values = (self.radius_m, self.speed_m_s, self.altitude_m, self.ramp_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("반경·속도·고도·램프 시간은 유한한 양수여야 합니다.")
        if not isinstance(self.turns, int) or isinstance(self.turns, bool) or self.turns < 1:
            raise ValueError("회전 횟수는 양의 정수여야 합니다.")
        if self.ramp_s > self.distance_m / self.speed_m_s:
            raise ValueError("경로 길이에 비해 가감속 시간이 너무 깁니다.")

    @property
    def direction(self):
        return 1.0 if self.clockwise else -1.0

    @property
    def distance_m(self):
        return 2*math.pi*self.radius_m*self.turns

    @property
    def duration_s(self):
        # 반코사인 가감속 구간의 평균 속도는 정속의 절반이다.
        return self.distance_m/self.speed_m_s + self.ramp_s

    def sample(self, elapsed_s: float) -> OrbitSample:
        if not math.isfinite(elapsed_s):
            raise ValueError("경과 시간은 유한해야 합니다.")
        t = min(self.duration_s, max(0.0, elapsed_s))
        ramp = self.ramp_s
        cruise_s = self.distance_m/self.speed_m_s - ramp
        if t < ramp:
            distance = self.speed_m_s/2*(t-ramp/math.pi*math.sin(math.pi*t/ramp))
            speed = self.speed_m_s/2*(1-math.cos(math.pi*t/ramp))
        elif t < ramp + cruise_s:
            distance = self.speed_m_s*(ramp/2+t-ramp)
            speed = self.speed_m_s
        elif t < self.duration_s:
            d = t-ramp-cruise_s
            distance = self.speed_m_s*(ramp/2+cruise_s) + self.speed_m_s/2*(
                d+ramp/math.pi*math.sin(math.pi*d/ramp))
            speed = self.speed_m_s/2*(1+math.cos(math.pi*d/ramp))
        else:
            distance, speed = self.distance_m, 0.0
        angle = self.direction*distance/self.radius_m
        omega = self.direction*speed/self.radius_m
        return OrbitSample(
            (self.radius_m*math.cos(angle), self.radius_m*math.sin(angle), -self.altitude_m),
            (-self.radius_m*math.sin(angle)*omega, self.radius_m*math.cos(angle)*omega, 0.0),
            angle, speed, elapsed_s >= self.duration_s,
        )


class OrbitProgress:
    """실제 위치의 atan2 경계를 풀어 부호 있는 회전각을 누적한다."""

    def __init__(self, min_radius_m: float, max_step_rad: float):
        self.min_radius_m = min_radius_m
        self.max_step_rad = max_step_rad
        self.previous_angle_rad = None
        self.angle_rad = 0.0

    def update(self, position_mission) -> float:
        x, y = position_mission[:2]
        if not math.isfinite(x) or not math.isfinite(y) or math.hypot(x, y) < self.min_radius_m:
            raise ValueError("원주비행 실제 위치가 중심에 너무 가깝거나 무효입니다.")
        angle = math.atan2(y, x)
        if self.previous_angle_rad is not None:
            delta = wrap_angle(angle-self.previous_angle_rad)
            if abs(delta) > self.max_step_rad:
                raise ValueError("실제 회전각이 한 주기 동안 비정상적으로 변했습니다.")
            self.angle_rad += delta
        self.previous_angle_rad = angle
        return self.angle_rad
