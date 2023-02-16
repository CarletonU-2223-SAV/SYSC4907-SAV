import numpy as np
from common.bridge import get_bridge
from geometry_msgs.msg import Point32
import open3d as o3d
from sklearn.cluster import DBSCAN
from lidar_object_recognition.scripts.cluster_detection import find_aabb
import os


class LidarProcessor:
    def __init__(self):
        param_file = open(os.path.abspath(os.path.dirname(__file__)) + "/DBSCAN_variable.txt")
        self.bridge = get_bridge()
        self.min_samples = int(param_file.readline())
        self.eps = float(param_file.readline())

    def processor(self, lidar_points: [Point32]):
        o3d_point_cloud = o3d.geometry.PointCloud()
        translated_points = []
        for point in lidar_points:
            if point.z > 0.1:
                translated_points.append([point.x, point.y, point.z])

        o3d_point_cloud.points = o3d.utility.Vector3dVector(translated_points)
        dbscan = DBSCAN(eps=self.eps, min_samples=self.min_samples)

        dbscan.fit(o3d_point_cloud)
        labels = np.array(dbscan.labels_)

        cluster_dic = dict()
        index_counter = 0

        for index in labels:

            # "-1" refers to not being a part of any cluster
            if index == -1:
                continue

            if index not in cluster_dic:
                cluster_dic[index] = []

            cluster_dic[index].append(translated_points[index_counter])
            index_counter += 1

        bounding_boxes = []
        for value in cluster_dic.values():
            find_aabb(value, bounding_boxes)

        car_pos = self.bridge.get_position()
        bounding_boxes.append(car_pos.x_val.real)
        bounding_boxes.append(car_pos.y_val.real)
        bounding_boxes.append(car_pos.z_val.real)

        return bounding_boxes

