"""DDS 연결 없이 데이터 만료·timestamp 정지·추정 유효성 검증."""

import unittest
from types import SimpleNamespace

from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus

import config
from px4_interface import PX4Interface, ReceivedState


class ReceivedStateTests(unittest.TestCase):
    def test_missing_and_zero_timestamp(self):
        state = ReceivedState()
        self.assertEqual(state.problem(10.0), "미수신")
        state.update(VehicleLocalPosition(timestamp=0), 10.0)
        self.assertEqual(state.problem(10.0), "timestamp=0")

    def test_receipt_timeout(self):
        state = ReceivedState()
        state.update(VehicleLocalPosition(timestamp=1), 10.0)
        self.assertIsNone(state.problem(10.1))
        self.assertEqual(state.problem(10.0 + config.DATA_TIMEOUT_S + 0.1), "수신 중단")

    def test_duplicate_timestamp_does_not_refresh_data(self):
        state = ReceivedState()
        state.update(VehicleLocalPosition(timestamp=100), 10.0)
        now_s = 10.0 + config.DATA_TIMEOUT_S + 0.1
        state.update(VehicleLocalPosition(timestamp=100), now_s)
        self.assertEqual(state.problem(now_s), "timestamp 갱신 중단")
        state.update(VehicleLocalPosition(timestamp=101), now_s + 0.1)
        self.assertIsNone(state.problem(now_s + 0.1))


class EstimateValidityTests(unittest.TestCase):
    def setUp(self):
        # ROS Node 생성 없이 상태 검사에 필요한 세 수신 저장소만 구성한다.
        self.position = VehicleLocalPosition(
            timestamp=123, xy_valid=True, z_valid=True, heading_good_for_control=True,
        )
        self.status = VehicleStatus(timestamp=123, pre_flight_checks_pass=True)
        self.states = SimpleNamespace(
            local_position=ReceivedState(), vehicle_status=ReceivedState(),
            land_detected=ReceivedState(),
        )
        self.states.local_position.update(self.position, 10.0)
        self.states.vehicle_status.update(self.status, 10.0)
        self.states.land_detected.update(VehicleLandDetected(timestamp=123), 10.0)

    def problems(self):
        return PX4Interface.state_problems(self.states, 10.1)

    def test_valid_estimates(self):
        self.assertEqual(self.problems(), [])

    def test_heading_not_ready_as_in_user_sample(self):
        self.position.heading_good_for_control = False
        self.assertIn("heading 최종 정렬 미완료", self.problems())

    def test_nonfinite_position_and_heading(self):
        self.position.x = float("nan")
        self.position.heading = float("inf")
        self.assertIn("위치에 NaN/Inf", self.problems())
        self.assertIn("heading에 NaN/Inf", self.problems())

    def test_invalid_position(self):
        self.position.z_valid = False
        self.assertIn("위치 추정 무효", self.problems())

    def test_failsafe_and_preflight(self):
        self.status.failsafe = True
        self.status.pre_flight_checks_pass = False
        self.assertIn("PX4 failsafe 활성", self.problems())
        self.assertIn("PX4 preflight 미통과", self.problems())


if __name__ == "__main__":
    unittest.main()
