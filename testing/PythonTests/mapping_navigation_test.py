from pure_pursuit import CarState, PurePursuit, euler_from_quaternion
from map_information import Point, RoadSegment, Lane
from route import Route
from math import pi, sqrt, atan2, sin


class TestPurePursuit:
    def test_update_pos(self):
        state = CarState(2)
        point = Point((1, 1))
        state.update_pos(point, (1, 0.5, 1.5, 1))
        assert state.point == point
        assert state.rear_x == 1 + sqrt(2) / 2
        assert state.rear_y == 1 - sqrt(2) / 2

    def test_update_speed(self):
        state = CarState(0)
        state.update_speed(5)
        assert state.velocity == 5

    def test_calc_distance(self):
        state = CarState(0)
        assert state.calc_distance(3, 4) == 5
        assert state.calc_distance(5, 12) == 13
        assert state.calc_distance(8, 15) == 17

    def test_pure_pursuit_steer_control(self):
        path = [Point((1, 1)), Point((1, 2))]
        pure_pursuit = PurePursuit(3.0, 0.1, path)
        state = CarState(2.2)
        alpha = atan2(2, 2.1)
        delta = atan2(4.4 / 3 * sin(alpha), 1)
        assert pure_pursuit.pure_pursuit_steer_control(state) == (delta, 1)

    def test_search_target_index(self):
        path = [Point((1, 1)), Point((3, 2)), Point((10, 10))]
        pure_pursuit = PurePursuit(3.0, 0.1, path)
        state = CarState(0)
        assert pure_pursuit.search_target_index(state) == (1, 3.0)
        state.update_pos(Point((3, 1)), (1, 1, 1, 1))
        assert pure_pursuit.search_target_index(state) == (2, 3.0)

    def test_euler_from_quaternion(self):
        assert euler_from_quaternion((1, 0.5, 1.5, 1)) == 3 * pi / 4
        assert euler_from_quaternion((1, -0.5, -1.5, 1)) == -3 * pi / 4


class TestRoute:
    def test_get_curr_segment(self):
        route = Route()
        segment = RoadSegment(Lane([Point((1, 1))]), 0)
        route.road_segments = [segment]
        assert route.get_curr_segment() == segment

    def test_get_next_segment(self):
        route = Route()
        route.road_segments = [RoadSegment(Lane([Point((1, 1))]), 0)]
        assert route.get_next_segment() is None
        segment = RoadSegment(Lane([Point((2, 2))]), 1)
        route.road_segments.append(segment)
        assert route.get_next_segment() == segment

    def test_next(self):
        route = Route()
        segment1 = RoadSegment(Lane([Point((1, 1))]), 0)
        segment2 = RoadSegment(Lane([Point((2, 2))]), 1)
        route.road_segments = [segment1, segment2]
        assert route.get_curr_segment() == segment1
        route.next()
        assert route.get_curr_segment() == segment2
