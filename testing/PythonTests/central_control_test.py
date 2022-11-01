from range import Range

class TestRange:
    def test_overlaps(self):
        range1 = Range(1, 5)
        range2 = Range(4, 7)
        range3 = Range(6, 10)

        assert range1.overlaps(range2)
        assert range2.overlaps(range3)
        assert not range1.overlaps(range3)
