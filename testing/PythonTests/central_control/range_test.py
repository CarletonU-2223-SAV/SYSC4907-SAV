from src.central_control.scripts.range import Range


def test_overlaps():
    range1 = Range(1, 5)
    range2 = Range(4, 7)
    range3 = Range(6, 10)

    assert range1.overlaps(range2)
    assert range2.overlaps(range3)
    assert not range1.overlaps(range3)
