import unittest

import config
from landing_profile import make_plan


class LandingProfileTests(unittest.TestCase):
    def setUp(self):
        self.original = {"MPC_LAND_ALT1": 10.0, "MPC_LAND_ALT2": 5.0,
                         "MPC_LAND_SPEED": 0.7, "MPC_Z_VEL_MAX_DN": 1.5}

    def check_intermediate_limits(self, current, desired):
        for name, value in make_plan(current, desired):
            current[name] = value
            self.assertGreater(current["MPC_LAND_ALT1"], current["MPC_LAND_ALT2"])
            self.assertGreaterEqual(current["MPC_Z_VEL_MAX_DN"], current["MPC_LAND_SPEED"])
        self.assertEqual(current, desired)

    def test_apply_and_restore_order_preserve_limits(self):
        self.check_intermediate_limits(dict(self.original), config.LANDING_PROFILE)
        self.check_intermediate_limits(dict(config.LANDING_PROFILE), self.original)

    def test_speed_below_documented_minimum_is_rejected(self):
        desired = dict(config.LANDING_PROFILE, MPC_LAND_SPEED=0.3)
        with self.assertRaises(ValueError):
            make_plan(self.original, desired)

    def test_no_unsupported_parameter_writes(self):
        desired = dict(config.LANDING_PROFILE, COM_DISARM_LAND=-1)
        with self.assertRaises(ValueError):
            make_plan(self.original, desired)


if __name__ == "__main__":
    unittest.main()
