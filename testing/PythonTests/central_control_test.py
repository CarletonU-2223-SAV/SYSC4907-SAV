from range import Range
from point import Point
from aabb import AABB
from math import sqrt


class TestRange:
    def test_overlaps(self):
        range1 = Range(1, 5)
        range2 = Range(4, 7)
        range3 = Range(6, 10)

        assert range1.overlaps(range2)
        assert range2.overlaps(range3)
        assert not range1.overlaps(range3)


class TestAABB:
    def test_intersection(self):
        aabb1 = AABB(Point(1, 3, 5), Point(4, 6, 8))
        aabb2 = AABB(Point(3, 5, 7), Point(5, 7, 9))
        aabb3 = AABB(Point(4, 6, 8), Point(7, 9, 11))

        assert aabb1.intersection(aabb2)
        assert aabb2.intersection(aabb3)
        assert not aabb3.intersection(aabb1)

    def test_vector_to_closest_point(self):
        aabb1 = AABB(Point(3, -2, 5), Point(5, 1, 8))
        aabb2 = AABB(Point(-4, 3, -3), Point(-1, 6, 1))
        aabb3 = AABB(Point(6, 10, 12), Point(9, 11, 15))

        assert aabb1.vector_to_closest_point() == (sqrt(3 ** 2 + 1 ** 2 + 5 ** 2), [3, 1, 5])
        assert aabb2.vector_to_closest_point() == (sqrt((-1) ** 2 + 3 ** 2 + 1 ** 2), [-1, 3, 1])
        assert aabb3.vector_to_closest_point() == (sqrt(6 ** 2 + 10 ** 2 + 12 ** 2), [6, 10, 12])
