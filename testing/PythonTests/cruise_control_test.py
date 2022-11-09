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

        assert pid.update_pid_output(4, 2) == 1 - exp(-1 * 4.41875)
        assert pid.speed_difference == 1.75
        assert pid.speed_difference_sum == 3.5
        assert pid.last_error == 1.75
        assert pid.kI == 0.75

        assert pid.update_pid_output(5, 1.5) == 1 - exp(-1 * 2639 / 480)
        assert pid.speed_difference == 0.75
        assert pid.speed_difference_sum == 6.375
        assert pid.last_error == 0.75
        assert pid.kI == 0.75
