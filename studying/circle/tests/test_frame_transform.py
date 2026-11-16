"""ROS 없이 좌표 부호, 기준점, 역변환, 90도 회전 후 전진을 검증."""

import math
import random
import unittest

from frame_transform import (
    MissionFrame, capture_relative_target_ned, heading_frd_displacement_to_ned, wrap_angle,
)


class FrameTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=10)

    def test_nonzero_origin_north_heading(self):
        frame = MissionFrame((12.0, -3.0, -0.04), 0.0)
        self.assertVector(frame.mission_to_ned((4, 0, -4)), (16, -3, -4.04))

    def test_east_heading_forward_and_right(self):
        frame = MissionFrame((12, -3, 2), math.pi/2)
        self.assertVector(frame.mission_to_ned((4, 0, -4)), (12, 1, -2))
        self.assertVector(frame.mission_to_ned((0, 4, 0)), (8, -3, 2))

    def test_current_heading_is_separate_from_fixed_mission_heading(self):
        frame = MissionFrame((10, 20, 0), 0.0)
        self.assertVector(frame.mission_to_ned((1, 0, 0)), (11, 20, 0))
        self.assertVector(capture_relative_target_ned((10, 20, -4), math.pi/2, (1, 0, 0)),
                          (10, 21, -4))
        # 저장한 목표는 기체가 그 뒤 회전하거나 이동해도 바뀌지 않는다.
        target = capture_relative_target_ned((10, 20, -4), math.pi/2, (1, 0, 0))
        self.assertVector(target, (10, 21, -4))

    def test_cardinal_headings(self):
        for yaw, expected in ((0, (1, 0, 0)), (math.pi/2, (0, 1, 0)),
                              (math.pi, (-1, 0, 0)), (-math.pi/2, (0, -1, 0))):
            with self.subTest(yaw=yaw):
                self.assertVector(heading_frd_displacement_to_ned((1, 0, 0), yaw), expected)

    def test_round_trip_and_distance_preservation(self):
        rng = random.Random(42)
        for _ in range(100):
            frame = MissionFrame(tuple(rng.uniform(-100, 100) for _ in range(3)),
                                 rng.uniform(-math.pi, math.pi))
            position = tuple(rng.uniform(-10, 10) for _ in range(3))
            transformed = frame.mission_to_ned(position)
            self.assertVector(frame.ned_to_mission(transformed), position)
            self.assertAlmostEqual(math.dist(transformed, frame.origin_ned), math.sqrt(sum(v*v for v in position)))

    def test_yaw_boundary_and_round_trip(self):
        self.assertAlmostEqual(wrap_angle(math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(3*math.pi), -math.pi)
        frame = MissionFrame((0, 0, 0), math.radians(170))
        self.assertAlmostEqual(frame.mission_yaw_to_ned(math.radians(30)), math.radians(-160))
        self.assertAlmostEqual(frame.ned_yaw_to_mission(math.radians(-160)), math.radians(30))

    def test_reject_nonfinite_inputs(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                MissionFrame((0, 0, bad), 0)
            with self.assertRaises(ValueError):
                wrap_angle(bad)
        with self.assertRaises(ValueError):
            MissionFrame((1, 2), 0)


if __name__ == "__main__":
    unittest.main()
