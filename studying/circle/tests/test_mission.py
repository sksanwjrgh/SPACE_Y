"""ROS 없이 정상 미션과 제어권 인계/중단 조건을 검증한다."""

from dataclasses import replace
import unittest

import config as c
from mission import FlightMission, Telemetry
from frame_transform import MissionFrame


GROUND = Telemetry(
    fresh=True, status_fresh=True, position_valid=True,
    position_ned=(12.0, -3.0, -0.04), velocity_ned=(0.0, 0.0, 0.0),
    yaw_ned_rad=1.67, armed=False, landed=True, nav_state=4,
    preflight=True, failsafe=False, resets=(4, 3, 1), delta_heading_rad=0.0,
)


class MissionTests(unittest.TestCase):
    def airborne(self):
        m = FlightMission(0.0)
        m.state = "HOVER"
        m.origin_ned = GROUND.position_ned
        m.frame = MissionFrame(GROUND.position_ned, GROUND.yaw_ned_rad)
        m.target_ned = (12.0, -3.0, -4.04)
        m.yaw_ned_rad = GROUND.yaw_ned_rad
        m.resets = GROUND.resets
        m.ever_armed = True
        t = replace(GROUND, armed=True, landed=False, nav_state=14,
                    position_ned=m.target_ned)
        return m, t

    def test_full_mission_actual_status_required_and_landing_handover(self):
        m = FlightMission(0.0)
        t = GROUND
        visited = set()
        commands = []
        for index in range(2500):
            now = index*c.CONTROL_PERIOD_S
            out = m.tick(now, t)
            visited.add(m.state)
            if out.command:
                commands.append(out.command)
                if out.command == "OFFBOARD":
                    self.assertFalse(t.armed)
                    t = replace(t, nav_state=14)
                elif out.command == "ARM":
                    self.assertEqual(t.nav_state, 14)
                    t = replace(t, armed=True)
                elif out.command == "LAND":
                    t = replace(t, nav_state=18)
            if out.position_ned and m.state in ("TAKEOFF", "HOVER"):
                t = replace(t, position_ned=out.position_ned,
                            landed=out.position_ned[2] >= GROUND.position_ned[2] - 0.1)
            elif m.state == "LAND" and t.nav_state == 18 and out.command != "LAND":
                self.assertIsNone(out.position_ned)
                t = replace(t, landed=True, armed=False, position_ned=GROUND.position_ned)
            if m.finished:
                break
        self.assertEqual(m.state, "DONE")
        self.assertFalse(m.failure)
        self.assertTrue({"WAIT", "PREPARE_OFFBOARD", "OFFBOARD", "ARM", "TAKEOFF", "HOVER", "LAND", "DONE"} <= visited)
        self.assertEqual(commands, ["OFFBOARD", "ARM", "LAND"])
        self.assertAlmostEqual(m.max_altitude_m, 4.0)

    def test_no_arm_without_actual_offboard_status(self):
        m = FlightMission(0.0)
        commands = []
        for i in range(600):
            out = m.tick(i*c.CONTROL_PERIOD_S, GROUND)
            commands.append(out.command)
            if m.finished:
                break
        self.assertNotIn("ARM", commands)
        self.assertEqual(m.state, "FAILED")

    def test_frame_flight_rotates_then_captures_one_fixed_forward_goal(self):
        m = FlightMission(0.0, frame_test=True)
        t = GROUND
        visited = set()
        forward_goals = []
        for index in range(4000):
            out = m.tick(index*c.CONTROL_PERIOD_S, t)
            visited.add(m.state)
            if out.command == "OFFBOARD":
                t = replace(t, nav_state=14)
            elif out.command == "ARM":
                t = replace(t, armed=True)
            elif out.command == "LAND":
                t = replace(t, nav_state=18)
            if m.state == "FORWARD_TEST":
                forward_goals.append(m.segment_goal_ned)
            if out.position_ned is not None and m.state not in ("WAIT", "PREPARE_OFFBOARD", "OFFBOARD", "ARM"):
                t = replace(t, position_ned=out.position_ned, yaw_ned_rad=out.yaw_ned_rad,
                            landed=False)
            elif m.state == "LAND" and t.nav_state == 18:
                t = replace(t, position_ned=GROUND.position_ned, armed=False, landed=True)
            if m.finished:
                break
        self.assertEqual(m.state, "DONE")
        self.assertFalse(m.failure)
        self.assertTrue({"YAW_TEST", "FORWARD_TEST", "RETURN_TEST"} <= visited)
        self.assertTrue(forward_goals)
        self.assertTrue(all(goal == forward_goals[0] for goal in forward_goals))
        from frame_transform import heading_frd_displacement_to_ned, wrap_angle
        delta = tuple(g-s for g, s in zip(forward_goals[0], m.forward_start_ned))
        movement = heading_frd_displacement_to_ned(delta, -m.forward_heading_ned_rad)
        self.assertAlmostEqual(movement[0], c.FRAME_TEST_FORWARD_M)
        self.assertAlmostEqual(movement[1], 0)
        self.assertAlmostEqual(wrap_angle(m.forward_heading_ned_rad - GROUND.yaw_ned_rad), c.FRAME_TEST_YAW_RAD)

    def test_frame_test_respects_external_mode_change(self):
        for phase in ("YAW_TEST", "FORWARD_TEST", "RETURN_TEST"):
            with self.subTest(phase=phase):
                m, t = self.airborne()
                m.state = phase
                out = m.tick(0.05, replace(t, nav_state=2))
                self.assertEqual(m.state, "FAILED")
                self.assertIsNone(out.position_ned)
                self.assertIsNone(out.command)

    def test_already_armed_at_start_is_not_taken_over(self):
        m = FlightMission(0.0)
        out = m.tick(0.05, replace(GROUND, armed=True))
        self.assertEqual(m.state, "FAILED")
        self.assertIsNone(out.command)
        self.assertIsNone(out.position_ned)

    def test_disarmed_offboard_can_prepare_but_never_arm_without_preflight(self):
        m = FlightMission(0.0)
        t = replace(GROUND, nav_state=14, preflight=False)
        saw_stream = False
        for index in range(500):
            out = m.tick(index*c.CONTROL_PERIOD_S, t)
            saw_stream |= out.position_ned is not None
            self.assertNotEqual(out.command, "ARM")
            if m.finished:
                break
        self.assertTrue(saw_stream)
        self.assertEqual(m.state, "FAILED")
        self.assertIn("preflight", m.failure)

    def test_unready_nonoffboard_stays_waiting(self):
        m = FlightMission(0.0)
        for index in range(50):
            out = m.tick(index*c.CONTROL_PERIOD_S, replace(GROUND, preflight=False))
            self.assertEqual(m.state, "WAIT")
            self.assertIsNone(out.position_ned)
            self.assertIsNone(out.command)

    def test_stale_connection_stops_heartbeat_and_commands(self):
        m, t = self.airborne()
        out = m.tick(0.05, replace(t, fresh=False, status_fresh=False))
        self.assertEqual(m.state, "LAND")
        self.assertIsNone(out.position_ned)
        self.assertIsNone(out.command)

    def test_invalid_position_requests_land_without_position_stream(self):
        m, t = self.airborne()
        out = m.tick(0.05, replace(t, position_valid=False))
        self.assertEqual(out.command, "LAND")
        self.assertIsNone(out.position_ned)

    def test_user_mode_change_is_respected(self):
        m, t = self.airborne()
        out = m.tick(0.05, replace(t, nav_state=2))
        self.assertEqual(m.state, "FAILED")
        self.assertIsNone(out.command)
        self.assertIsNone(out.position_ned)

    def test_user_abort_waits_for_landed_and_disarmed(self):
        m, t = self.airborne()
        m.abort("사용자 중단", t, 0.0)
        self.assertEqual(m.tick(0.05, t).command, "LAND")
        self.assertIsNone(m.tick(0.10, replace(t, nav_state=18)).position_ned)
        m.tick(0.15, replace(t, nav_state=18, landed=True))
        self.assertFalse(m.finished)
        m.tick(0.20, replace(t, nav_state=18, landed=True, armed=False))
        self.assertEqual(m.state, "DONE")
        self.assertEqual(m.failure, "사용자 중단")

    def test_ekf_position_reset_aborts(self):
        m, t = self.airborne()
        out = m.tick(0.05, replace(t, resets=(5, 3, 1)))
        self.assertEqual(m.state, "LAND")
        self.assertEqual(out.command, "LAND")

    def test_ekf_heading_reset_updates_only_yaw_target(self):
        m, t = self.airborne()
        out = m.tick(0.05, replace(t, resets=(4, 3, 2), delta_heading_rad=0.2))
        self.assertAlmostEqual(out.yaw_ned_rad, 1.87)
        self.assertEqual(m.state, "HOVER")

    def test_landing_timeout_is_not_success(self):
        m, t = self.airborne()
        m.abort("test", t, 0.0)
        m.tick(c.LAND_TIMEOUT_S + 1.0, replace(t, nav_state=18))
        self.assertEqual(m.state, "FAILED")

    def test_control_gap_aborts(self):
        m, t = self.airborne()
        m.tick(c.CONTROL_GAP_LIMIT_S + 0.1, t)
        self.assertEqual(m.state, "LAND")

    def prepare(self):
        m = FlightMission(0.0)
        for index in range(45):
            m.tick(index*c.CONTROL_PERIOD_S, GROUND)
        self.assertEqual(m.state, "PREPARE_OFFBOARD")
        return m

    def test_ground_preparation_gap_restarts_full_stream_before_mode_command(self):
        m = self.prepare()
        restart_s = m.last_tick_s + 0.335
        out = m.tick(restart_s, GROUND)
        self.assertEqual(m.state, "PREPARE_OFFBOARD")
        self.assertEqual(m.warmup_restarts, 1)
        self.assertIsNone(out.command)
        self.assertIsNotNone(out.position_ned)
        steps = round(c.PRESTREAM_S / c.CONTROL_PERIOD_S)
        for index in range(1, steps):
            out = m.tick(restart_s + index*c.CONTROL_PERIOD_S, GROUND)
            self.assertEqual(m.state, "PREPARE_OFFBOARD")
            self.assertIsNone(out.command)
        now_s = restart_s + (steps + 1)*c.CONTROL_PERIOD_S
        m.tick(now_s, GROUND)
        self.assertEqual(m.state, "OFFBOARD")
        self.assertEqual(m.tick(now_s + c.CONTROL_PERIOD_S, GROUND).command, "OFFBOARD")

    def test_repeated_preparation_gaps_eventually_fail_without_arm(self):
        m = self.prepare()
        for _ in range(100):
            out = m.tick(m.last_tick_s + 0.335, GROUND)
            self.assertIsNone(out.command)
            if m.finished:
                break
        self.assertEqual(m.state, "FAILED")
        self.assertIn("준비 스트림 안정화 시간 초과", m.failure)

    def test_gap_after_mode_request_does_not_restart_preparation(self):
        m = self.prepare()
        m.transition("OFFBOARD", m.last_tick_s)
        out = m.tick(m.last_tick_s + 0.335, GROUND)
        self.assertEqual(m.state, "FAILED")
        self.assertEqual(m.warmup_restarts, 0)
        self.assertIsNone(out.command)

    def test_armed_preparation_gap_is_not_retried(self):
        m = self.prepare()
        t = replace(GROUND, armed=True)
        m.tick(m.last_tick_s + 0.335, t)
        self.assertEqual(m.state, "LAND")
        self.assertEqual(m.warmup_restarts, 0)

    def test_stale_preparation_data_is_not_retried(self):
        m = self.prepare()
        m.tick(m.last_tick_s + 0.335, replace(GROUND, fresh=False))
        self.assertEqual(m.state, "FAILED")
        self.assertEqual(m.warmup_restarts, 0)

    def test_takeoff_ramp_is_bounded_and_smooth(self):
        m, t = self.airborne()
        m.state = "TAKEOFF"
        m.target_ned = m.origin_ned
        t = replace(t, position_ned=m.origin_ned, landed=True)
        prior_z = m.origin_ned[2]
        for index in range(int(c.TAKEOFF_RAMP_S/c.CONTROL_PERIOD_S) + 1):
            now = index*c.CONTROL_PERIOD_S
            out = m.tick(now, t)
            self.assertEqual(m.state, "TAKEOFF")
            self.assertLessEqual(out.position_ned[2], prior_z + 1e-9)
            self.assertLessEqual(abs(out.position_ned[2] - prior_z)/c.CONTROL_PERIOD_S, 0.63)
            prior_z = out.position_ned[2]
            t = replace(t, position_ned=out.position_ned, landed=False)
        self.assertAlmostEqual(prior_z, GROUND.position_ned[2] - 4)


if __name__ == "__main__":
    unittest.main()
