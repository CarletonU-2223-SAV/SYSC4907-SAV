
from pid_controller import PIDController
from math import exp


class TestPIDController:
    def test_pidToCarValues(self):
        pid = PIDController()

        assert pid.pidToCarValues(1) == 1 - exp(-1 * 1)
        assert pid.pidToCarValues(2) == 1 - exp(-1 * 2)
        assert pid.pidToCarValues(-1) == 0.0

    def test_update_target_speed(self):
        pid = PIDController()
        pid.update_target_speed(5)

        assert pid.target_speed == 5.75

    def test_update_pid_output(self):
        pid = PIDController()

        assert pid.update_pid_output(4)
