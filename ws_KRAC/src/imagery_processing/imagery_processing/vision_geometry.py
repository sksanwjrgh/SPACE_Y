import math
from typing import Optional, Tuple


class VisionGeometry:
    """Handles pinhole back-projection, attitude tilt de-rotation, and target loss watchdog."""

    def __init__(
        self,
        fx: float = 540.0,
        fy: float = 540.0,
        cx: float = 640.0,
        cy: float = 480.0,
        min_altitude: float = 0.1,
        timeout_sec: float = 0.3,
        max_tilt_rad: float = math.radians(30.0),
        use_attitude_compensation: bool = False,
    ) -> None:
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.min_altitude = float(min_altitude)
        self.timeout_sec = float(timeout_sec)
        self.max_tilt_rad = float(max_tilt_rad)
        self.use_attitude_compensation = bool(use_attitude_compensation)

        # Watchdog state
        self._last_x: float = float("nan")
        self._last_y: float = float("nan")
        self._last_timestamp: Optional[float] = None

    def project_to_ground(
        self,
        u: float,
        v: float,
        altitude: float,
        roll: float = 0.0,
        pitch: float = 0.0,
    ) -> Tuple[Optional[float], Optional[float]]:
        """Projects image coordinates (u, v) to metric ground frame (FRD)."""
        if altitude is None or math.isnan(altitude) or altitude < self.min_altitude:
            return None, None

        if not self.use_attitude_compensation:
            # Direct pinhole metric projection (avoids video transport phase lag)
            x_frd = ((self.cy - v) / self.fy) * altitude
            y_frd = ((u - self.cx) / self.fx) * altitude
            return x_frd, y_frd

        # Clamp attitude angles to prevent singularities
        r = max(-self.max_tilt_rad, min(self.max_tilt_rad, float(roll)))
        p = max(-self.max_tilt_rad, min(self.max_tilt_rad, float(pitch)))

        # Ray in camera frame: +x right, +y down, +z forward along optical axis
        # Camera mounted looking down:
        # Camera optical axis is Drone +Z (Down)
        # Camera +x (image right) is Drone +Y (Right)
        # Camera +y (image down) is Drone -X (Backward) -> image up (cy - v) is Drone +X (Forward)
        rx_cam = (u - self.cx) / self.fx
        ry_cam = (v - self.cy) / self.fy
        rz_cam = 1.0

        # Body frame (FRD):
        # r_body_x = -ry_cam = (self.cy - v) / self.fy
        # r_body_y = rx_cam = (u - self.cx) / self.fx
        # r_body_z = rz_cam = 1.0
        rbx = (self.cy - v) / self.fy
        rby = (u - self.cx) / self.fx
        rbz = 1.0

        # De-rotate from body frame to horizontal level frame.
        # Rotation Level -> Body: R = R_x(roll) * R_y(pitch)
        # Body -> Level: R_transpose = R_y(-pitch) * R_x(-roll)
        cos_p, sin_p = math.cos(p), math.sin(p)
        cos_r, sin_r = math.cos(r), math.sin(r)

        # First apply R_x(-r) around X:
        # x1 = rbx
        # y1 = cos_r * rby + sin_r * rbz
        # z1 = -sin_r * rby + cos_r * rbz
        x1 = rbx
        y1 = cos_r * rby + sin_r * rbz
        z1 = -sin_r * rby + cos_r * rbz

        # Then apply R_y(-p) around Y:
        # rlx = cos_p * x1 - sin_p * z1
        # rly = y1
        # rlz = sin_p * x1 + cos_p * z1
        # When drone pitches nose down (in test: camera tilts forward, ground moves down in image):
        # p > 0, delta_v > 0 -> rbx < 0.
        # rlx = cos_p * x1 + sin_p * z1 cancels when rbx = -tan(p) * rbz!
        rlx = cos_p * x1 + sin_p * z1
        rly = y1
        rlz = -sin_p * x1 + cos_p * z1

        # Check ray points downwards
        if rlz <= 0.2:
            return None, None

        # Intersect with ground plane at distance Z (altitude)
        scale = altitude / rlz
        x_frd = rlx * scale
        y_frd = rly * scale

        return x_frd, y_frd

    def update_detection(self, x_m: float, y_m: float, timestamp: float) -> None:
        """Records a valid marker detection."""
        self._last_x = float(x_m)
        self._last_y = float(y_m)
        self._last_timestamp = float(timestamp)

    def is_target_valid(self, current_time: float) -> bool:
        """Returns True if a valid marker was detected within timeout_sec."""
        if self._last_timestamp is None:
            return False
        return (current_time - self._last_timestamp) <= self.timeout_sec

    def get_target_coordinates(self, current_time: float) -> Tuple[float, float]:
        """Returns the latest coordinates if valid, or (NaN, NaN) if timed out."""
        if self.is_target_valid(current_time):
            return self._last_x, self._last_y
        return float("nan"), float("nan")
