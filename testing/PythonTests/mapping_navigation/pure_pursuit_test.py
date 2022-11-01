from src.mapping_navigation.scripts.map_information import *
from src.mapping_navigation.scripts import pure_pursuit


def car_state_update_pos():
    state = pure_pursuit.CarState(5)
    state.update_pos(Point((2, 2)), (4, 6, 8, 10))
    print(f'{state.rear_x}, {state.rear_y}, {state.yaw}')


if __name__ == '__main__':
    car_state_update_pos()
