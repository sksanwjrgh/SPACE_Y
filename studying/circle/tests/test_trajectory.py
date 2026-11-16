"""원 궤적의 기하·속도·yaw·실제 회전각 누적을 독립적으로 검증한다."""

import math
import unittest

from frame_transform import MissionFrame, heading_frd_displacement_to_ned, wrap_angle
from trajectory import CircleTrajectory, OrbitProgress, inward_yaw_ned, line_position


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.circle = CircleTrajectory(4.0, 0.5, 2, 4.0)

    def test_full_distance_time_and_endpoint(self):
        circle = self.circle
        self.assertAlmostEqual(circle.duration_s, 4*math.pi*4/0.5 + 3)
        end = circle.sample(circle.duration_s)
        self.assertTrue(end.finished)
        self.assertAlmostEqual(end.angle_rad, 4*math.pi)
        self.assertAlmostEqual(end.position_mission[0], 4)
        self.assertAlmostEqual(end.position_mission[1], 0)
        self.assertEqual(end.speed_m_s, 0)
        self.assertEqual(circle.sample(-10).speed_m_s, 0)

    def test_clockwise_quarter_circle_and_counterclockwise(self):
        quarter_time = self.circle.ramp_s/2 + (math.pi*4/2)/0.5
        clockwise = self.circle.sample(quarter_time)
        other = CircleTrajectory(4, 0.5, 2, 4, clockwise=False).sample(quarter_time)
        self.assertAlmostEqual(clockwise.position_mission[0], 0)
        self.assertAlmostEqual(clockwise.position_mission[1], 4)
        self.assertAlmostEqual(other.position_mission[1], -4)

    def test_radius_altitude_speed_and_velocity_are_consistent(self):
        for index in range(1, 1000):
            t = self.circle.duration_s*index/1000
            sample = self.circle.sample(t)
            x, y, z = sample.position_mission
            vx, vy, vz = sample.velocity_mission
            self.assertAlmostEqual(math.hypot(x, y), 4)
            self.assertEqual(z, -4)
            self.assertAlmostEqual(x*vx+y*vy, 0)
            self.assertLessEqual(sample.speed_m_s, 0.5)
            self.assertGreaterEqual(sample.speed_m_s, 0)
            h = 1e-4
            before, after = self.circle.sample(t-h), self.circle.sample(t+h)
            self.assertAlmostEqual((after.position_mission[0]-before.position_mission[0])/(2*h), vx, places=6)
            self.assertAlmostEqual((after.position_mission[1]-before.position_mission[1])/(2*h), vy, places=6)

    def test_acceleration_and_deceleration_boundaries_are_continuous(self):
        c = self.circle
        for t in (c.ramp_s, c.duration_s-c.ramp_s, c.duration_s):
            before, after = c.sample(t-1e-6), c.sample(t+1e-6)
            self.assertLess(math.dist(before.position_mission, after.position_mission), 2e-6)
            self.assertAlmostEqual(before.speed_m_s, after.speed_m_s, places=6)

    def test_center_yaw_uses_actual_position_and_matches_mission_formula(self):
        frame = MissionFrame((12, -8, 0.2), 1.23)
        actual_mission = (3.8, 0.4, -4)
        actual_ned = frame.mission_to_ned(actual_mission)
        direct = inward_yaw_ned(actual_ned, frame.origin_ned, 0)
        through_mission = frame.mission_yaw_to_ned(math.atan2(-0.4, -3.8))
        self.assertAlmostEqual(wrap_angle(direct-through_mission), 0)
        self.assertNotAlmostEqual(wrap_angle(direct-frame.mission_yaw_to_ned(math.pi)), 0)
        self.assertEqual(inward_yaw_ned(frame.origin_ned, frame.origin_ned, 0.3), wrap_angle(0.3))

    def test_inward_clockwise_motion_is_to_heading_left(self):
        sample = self.circle.sample(20)
        yaw = inward_yaw_ned(sample.position_mission, (0, 0, 0), 0)
        body_velocity = heading_frd_displacement_to_ned(sample.velocity_mission, -yaw)
        self.assertAlmostEqual(body_velocity[0], 0)
        self.assertAlmostEqual(body_velocity[1], -0.5)

    def test_signed_actual_angle_unwraps_two_turns_and_reverse_motion(self):
        for direction in (1, -1):
            progress = OrbitProgress(1.0, math.radians(30))
            for index in range(801):
                theta = direction*4*math.pi*index/800
                progress.update((4*math.cos(theta), 4*math.sin(theta), -4))
            self.assertAlmostEqual(progress.angle_rad, direction*4*math.pi)
        progress = OrbitProgress(1.0, 0.5)
        for theta in (0, 0.2, 0.1):
            progress.update((4*math.cos(theta), 4*math.sin(theta)))
        self.assertAlmostEqual(progress.angle_rad, 0.1)

    def test_progress_rejects_center_and_angle_jump(self):
        progress = OrbitProgress(1, 0.5)
        with self.assertRaises(ValueError):
            progress.update((0, 0, -4))
        progress.update((4, 0, -4))
        with self.assertRaises(ValueError):
            progress.update((-4, 0, -4))

    def test_invalid_orbit_parameters(self):
        for radius, speed, turns in ((0, .5, 2), (4, 0, 2), (4, .5, 0), (4, .5, 1.5)):
            with self.assertRaises(ValueError):
                CircleTrajectory(radius, speed, turns, 4)
        with self.assertRaises(ValueError):
            CircleTrajectory(1, 10, 1, 4, ramp_s=3)

    def test_line_holds_endpoints(self):
        self.assertEqual(line_position((0, 0, -4), (4, 0, -4), -1, 15), (0, 0, -4))
        self.assertEqual(line_position((0, 0, -4), (4, 0, -4), 100, 15), (4, 0, -4))


if __name__ == '__main__':
    unittest.main()
