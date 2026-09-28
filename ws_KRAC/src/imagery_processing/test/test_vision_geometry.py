import math
import numpy as np
import pytest

from imagery_processing.vision_geometry import VisionGeometry


def test_pinhole_projection_nominal():
    geom = VisionGeometry(fx=540.0, fy=540.0, cx=640.0, cy=480.0, min_altitude=0.1)
    # Target 100px right, 50px up from center at altitude 5.0m
    u = 640.0 + 100.0
    v = 480.0 - 50.0
    altitude = 5.0

    x_frd, y_frd = geom.project_to_ground(u=u, v=v, altitude=altitude, roll=0.0, pitch=0.0)

    expected_x = (50.0 * 5.0) / 540.0
    expected_y = (100.0 * 5.0) / 540.0

    assert math.isclose(x_frd, expected_x, rel_tol=1e-3)
    assert math.isclose(y_frd, expected_y, rel_tol=1e-3)


def test_pinhole_projection_invalid_altitude():
    geom = VisionGeometry(fx=540.0, fy=540.0, cx=640.0, cy=480.0, min_altitude=0.1)
    
    # Altitude <= min_altitude or NaN
    assert geom.project_to_ground(u=700.0, v=400.0, altitude=0.05, roll=0.0, pitch=0.0) == (None, None)
    assert geom.project_to_ground(u=700.0, v=400.0, altitude=-1.0, roll=0.0, pitch=0.0) == (None, None)
    assert geom.project_to_ground(u=700.0, v=400.0, altitude=float("nan"), roll=0.0, pitch=0.0) == (None, None)


def test_attitude_derotation_pitch():
    geom = VisionGeometry(
        fx=540.0, fy=540.0, cx=640.0, cy=480.0, min_altitude=0.1, use_attitude_compensation=True
    )
    altitude = 5.0
    pitch = math.radians(10.0)  # nose down 10 degrees

    # When pitching nose down, the ground point at (0, 0) appears down in the image
    # Angle from camera optical axis is 10 degrees forward, so in image v increases:
    delta_v = 540.0 * math.tan(pitch)
    u = 640.0
    v = 480.0 + delta_v

    # With roll=0, pitch=10 deg, de-rotation should recover (X_frd ≈ 0, Y_frd ≈ 0)
    x_frd, y_frd = geom.project_to_ground(u=u, v=v, altitude=altitude, roll=0.0, pitch=pitch)

    assert abs(x_frd) < 0.05  # within 5cm of true center
    assert abs(y_frd) < 0.05


def test_watchdog_timeout():
    geom = VisionGeometry(timeout_sec=0.3)
    
    # Update with detection at t=1.0
    geom.update_detection(x_m=0.5, y_m=0.2, timestamp=1.0)
    assert geom.is_target_valid(current_time=1.1) is True
    x, y = geom.get_target_coordinates(current_time=1.1)
    assert math.isclose(x, 0.5)
    assert math.isclose(y, 0.2)

    # After 0.35s without update (t=1.35), target must be invalid and return NaN
    assert geom.is_target_valid(current_time=1.35) is False
    x_lost, y_lost = geom.get_target_coordinates(current_time=1.35)
    assert math.isnan(x_lost)
    assert math.isnan(y_lost)

    # Recover target with new detection at t=1.5
    geom.update_detection(x_m=0.1, y_m=0.2, timestamp=1.5)
    assert geom.is_target_valid(current_time=1.55) is True
    x_rec, y_rec = geom.get_target_coordinates(current_time=1.55)
    assert math.isclose(x_rec, 0.1)
    assert math.isclose(y_rec, 0.2)


def test_attitude_clamp_extreme_tilt():
    geom = VisionGeometry(
        fx=540.0, fy=540.0, cx=640.0, cy=480.0, min_altitude=1.0, use_attitude_compensation=True
    )
    # 60 degrees tilt should be clamped to 30 degrees (max_tilt_rad)
    x1, y1 = geom.project_to_ground(u=640.0, v=480.0, altitude=2.0, roll=0.0, pitch=math.radians(60.0))
    x2, y2 = geom.project_to_ground(u=640.0, v=480.0, altitude=2.0, roll=0.0, pitch=math.radians(30.0))
    assert math.isclose(x1, x2, rel_tol=1e-4)
    assert math.isclose(y1, y2, rel_tol=1e-4)

