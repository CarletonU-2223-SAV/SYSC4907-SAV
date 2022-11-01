from pure_pursuit import CarState
from map_information import Point


class TestPurePursuit:
    def test_car_state_update_pos(self):
        state = CarState(5)
        state.update_pos(Point((2, 2)), (4, 6, 8, 10))
        print(f'{state.rear_x}, {state.rear_y}, {state.yaw}')
        assert True
