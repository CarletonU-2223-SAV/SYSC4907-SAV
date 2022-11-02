#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import airsim


class Lidar:

    """
    Lidar data
    """

    def __init__(self):
        self.lidar_pub = rospy.Publisher('lidar', PointCloud ,queue_size=10)

    def send_lidar_data(self):
        rospy.init_node('talker', anonymous=True)

        host_ip = rospy.get_param('/host_ip')

        client = airsim.CarClient(ip=host_ip)
        client.confirmConnection()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            point_cloud = PointCloud()
            point_cloud.points = []

            lidar_data = client.getLidarData().point_cloud

            # Every point is made of 3 floats
            num_points = len(lidar_data) // 3

            # Convert format of points used by lidar to format of point used by PointCloud message
            for x in range(0, num_points):
                point = Point32(lidar_data[x * 3], lidar_data[x * 3 + 1], lidar_data[x * 3 + 2])
                point_cloud.points.append(point)

            self.lidar_pub.publish(point_cloud)
            rate.sleep()


if __name__ == '__main__':
    lidar = Lidar()
    lidar.send_lidar_data()