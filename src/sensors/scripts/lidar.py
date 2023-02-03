#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from common.bridge import get_bridge


FLOATS_PER_POINT = 3


class Lidar:
    """
    Lidar data
    """

    def __init__(self):
        self.lidar_pub = rospy.Publisher('lidar', PointCloud, queue_size=10)

    def send_lidar_data(self):
        rospy.init_node('talker', anonymous=True)
        sim = get_bridge()

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():

            point_cloud = PointCloud()
            point_cloud.points = []

            lidar_data = sim.get_lidar_point_cloud()

            num_points = len(lidar_data) // FLOATS_PER_POINT

            # Convert format of points used by lidar to format of point used by PointCloud message
            for x in range(0, num_points):
                offset = x * FLOATS_PER_POINT
                point = Point32(lidar_data[offset], lidar_data[offset + 1], lidar_data[offset + 2])
                point_cloud.points.append(point)

            self.lidar_pub.publish(point_cloud)
            rate.sleep()


if __name__ == '__main__':
    lidar = Lidar()
    lidar.send_lidar_data()