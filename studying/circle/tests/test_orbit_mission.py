"""원주비행의 전체 절차와 시간만으로 완료되지 않는 조건을 검증한다."""

from dataclasses import replace
import math
import unittest
from unittest.mock import patch

import config as c
from frame_transform import MissionFrame, wrap_angle
from mission import FlightMission
from trajectory import OrbitProgress
from test_mission import GROUND


class OrbitMissionTests(unittest.TestCase):
    def initialized_orbit(self):
        m = FlightMission(0, orbit=True)
        m.origin_ned = GROUND.position_ned
        m.frame = MissionFrame(GROUND.position_ned, GROUND.yaw_ned_rad)
        m.target_ned = m.frame.mission_to_ned((4, 0, -4))
        m.yaw_ned_rad = m.frame.mission_yaw_to_ned(math.pi)
        m.resets = GROUND.resets
        m.ever_armed = True
        m.state = 'ORBIT'
        m.orbit_progress = OrbitProgress(c.ORBIT_MIN_VALID_RADIUS_M, c.ORBIT_MAX_ANGLE_STEP_RAD)
        m.orbit_progress.update((4, 0, -4))
        t = replace(GROUND, position_ned=m.target_ned, armed=True, landed=False,
                    nav_state=14, yaw_ned_rad=m.yaw_ned_rad)
        return m, t

    def simulate(self, fixed_yaw=False, clockwise=True, heading=1.67):
        with patch.object(c, 'ORBIT_CLOCKWISE', clockwise):
            m = FlightMission(0, orbit=True, fixed_yaw=fixed_yaw)
        t = replace(GROUND, yaw_ned_rad=heading)
        visited = []
        orbit_yaws = []
        for index in range(7000):
            now = index*c.CONTROL_PERIOD_S
            out = m.tick(now, t)
            if not visited or visited[-1] != m.state:
                visited.append(m.state)
            if out.command == 'OFFBOARD':
                t = replace(t, nav_state=14)
            elif out.command == 'ARM':
                t = replace(t, armed=True)
            elif out.command == 'LAND':
                t = replace(t, nav_state=18)
            if m.state == 'ORBIT':
                orbit_yaws.append(out.yaw_ned_rad)
                self.assertLessEqual(math.dist(m.target_ned, t.position_ned), .5)
                if not fixed_yaw:
                    actual_inward = math.atan2(m.origin_ned[1]-t.position_ned[1],
                                              m.origin_ned[0]-t.position_ned[0])
                    self.assertLess(abs(wrap_angle(out.yaw_ned_rad-actual_inward)), 1e-8)
            if out.position_ned is not None and t.armed:
                # 1차 지연 모델: 위치 목표에 즉시 도달했다고 가정하지 않는다.
                velocity = tuple(3*(goal-actual) for goal, actual in zip(out.position_ned, t.position_ned))
                position = tuple(actual+v*c.CONTROL_PERIOD_S for actual, v in zip(t.position_ned, velocity))
                yaw = wrap_angle(t.yaw_ned_rad+min(1, 4*c.CONTROL_PERIOD_S)*wrap_angle(out.yaw_ned_rad-t.yaw_ned_rad))
                t = replace(t, position_ned=position, velocity_ned=velocity, yaw_ned_rad=yaw,
                            landed=position[2] >= GROUND.position_ned[2]-.05)
            elif m.state == 'LAND' and t.nav_state == 18:
                t = replace(t, position_ned=GROUND.position_ned, armed=False, landed=True, velocity_ned=(0, 0, 0))
            if m.finished:
                break
        self.assertEqual(m.state, 'DONE', m.failure)
        self.assertFalse(m.failure)
        expected = ['WAIT', 'PREPARE_OFFBOARD', 'OFFBOARD', 'ARM', 'TAKEOFF', 'HOVER',
                    'FORWARD', 'TURN_INWARD', 'ORBIT', 'ORBIT_SETTLE', 'RETURN', 'LAND', 'DONE']
        self.assertEqual(visited, expected)
        self.assertAlmostEqual(m.circle.direction*m.actual_angle_rad/(2*math.pi), m.circle.turns, delta=.01)
        if fixed_yaw:
            self.assertTrue(all(abs(wrap_angle(yaw-orbit_yaws[0])) < 1e-9 for yaw in orbit_yaws))
        return m

    def test_full_two_turn_inward_mission_with_tracking_delay(self):
        self.simulate()

    def test_fixed_yaw_one_turn_validation_procedure(self):
        m = self.simulate(fixed_yaw=True)
        self.assertEqual(m.circle.turns, 1)

    def test_counterclockwise_and_non_north_initial_heading(self):
        self.simulate(clockwise=False, heading=-2.1)

    def test_elapsed_time_does_not_prove_actual_turns(self):
        m, t = self.initialized_orbit()
        now = m.circle.duration_s
        for _ in range(round(c.ORBIT_COMPLETION_TIMEOUT_S/c.CONTROL_PERIOD_S)+3):
            m.last_tick_s = now-c.CONTROL_PERIOD_S
            m.tick(now, t)
            self.assertNotIn(m.state, ('ORBIT_SETTLE', 'RETURN', 'DONE'))
            now += c.CONTROL_PERIOD_S
            if m.state == 'LAND':
                break
        self.assertEqual(m.actual_angle_rad, 0)
        self.assertEqual(m.state, 'LAND')
        self.assertIn('완료 확인 시간 초과', m.failure)

    def test_near_center_aborts(self):
        m, t = self.initialized_orbit()
        out = m.tick(.05, replace(t, position_ned=m.frame.mission_to_ned((0, 0, -4))))
        self.assertEqual(m.state, 'LAND')
        self.assertEqual(out.command, 'LAND')

    def test_heading_reset_during_horizontal_mission_aborts(self):
        m, t = self.initialized_orbit()
        m.tick(.05, replace(t, resets=(4, 3, 2), delta_heading_rad=.1))
        self.assertEqual(m.state, 'LAND')
        self.assertIn('heading 재설정', m.failure)

    def test_offboard_loss_is_respected_in_each_phase(self):
        for phase in ('FORWARD', 'TURN_INWARD', 'ORBIT', 'ORBIT_SETTLE', 'RETURN'):
            m, t = self.initialized_orbit()
            m.state = phase
            out = m.tick(.05, replace(t, nav_state=2))
            self.assertEqual(m.state, 'FAILED')
            self.assertIsNone(out.command)
            self.assertIsNone(out.position_ned)


if __name__ == '__main__':
    unittest.main()
