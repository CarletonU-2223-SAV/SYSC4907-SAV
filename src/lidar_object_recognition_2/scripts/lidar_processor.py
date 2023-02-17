import numpy as np
from common.bridge import get_bridge
from geometry_msgs.msg import Point32
import open3d as o3d
from sklearn.cluster import DBSCAN
import os

default_min_x = 1e+308
default_max_x = -1e+308
default_min_y = 1e+308
default_max_y = -1e+308
default_min_z = 1e+308
default_max_z = -1e+308


def find_aabb(points: [float], result: [float]):

    # Max value of floats ensures that when iterating over points of the
    # bounding box, the points that comprise the box will be modified to
    # reflect the true min and max of the box
    min_x = default_min_x
    max_x = default_max_x

    min_y = default_min_y
    max_y = default_max_y

    min_z = default_min_z
    max_z = default_max_z

    for p in points:
        min_x = min(p[0], min_x)
        max_x = max(p[0], max_x)

        min_y = min(p[1], min_y)
        max_y = max(p[1], max_y)

        min_z = min(p[2], min_z)
        max_z = max(p[2], max_z)

    result.append(min_x)
    result.append(min_y)
    result.append(min_z)

    result.append(max_x)
    result.append(max_y)
    result.append(max_z)


class LidarProcessor:

    def __init__(self):
        param_file = open(os.path.abspath(os.path.dirname(__file__)) + "/DBSCAN_variable.txt")
        self.bridge = get_bridge()
        self.min_samples = int(param_file.readline())
        self.eps = float(param_file.readline())

    def processor(self, lidar_points: [Point32]) -> [float]:
        o3d_point_cloud = o3d.geometry.PointCloud()
        translated_points = []
        for point in lidar_points:
            if point.z > 0.1:
                translated_points.append([point.x, point.y, point.z])

        o3d_point_cloud.points = o3d.utility.Vector3dVector(translated_points)
        dbscan = DBSCAN(eps=self.eps, min_samples=self.min_samples)
        np_arr_cloud_point = np.array(o3d_point_cloud.points)
        labels = dbscan.fit_predict(np_arr_cloud_point)

        clusters = []
        for label in set(labels):

            # "-1" refers to not being a part of any cluster
            if label == -1:
                continue
            cluster = np_arr_cloud_point[labels == label]
            clusters.append(cluster)

        bounding_boxes = []
        for value in clusters:
            find_aabb(value, bounding_boxes)

        car_pos = self.bridge.get_position()
        bounding_boxes.append(car_pos.x_val.real)
        bounding_boxes.append(car_pos.y_val.real)
        bounding_boxes.append(car_pos.z_val.real)

        return bounding_boxes
