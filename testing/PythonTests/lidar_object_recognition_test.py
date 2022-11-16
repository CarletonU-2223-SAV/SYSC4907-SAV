from cluster_detection import ClusterDetection


class TestClusterDetection:
    def test_find_aabb(self):
        c_detection = ClusterDetection()
        result = []
        c_detection.find_aabb([[1, 2, 3], [3, 4, 5], [6, 7, 8]], result)
        assert result == [1, 2, 3, 6, 7, 8]


