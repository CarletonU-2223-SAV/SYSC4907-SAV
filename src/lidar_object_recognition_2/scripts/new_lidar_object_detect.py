import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray

from lidar_processor import LidarProcessor

q_size = 20


class NewLidarObjectHandler:

    def __init__(self):
        self.lidarPub = rospy.Publisher("new_lidar_data", Float64MultiArray, queue_size=q_size)
        self.lidar_processor = LidarProcessor()

    def listener(self):
        rospy.init_node("new_lidar_object_detect", anonymous=True)
        rospy.Subscriber("lidar", PointCloud, self.lidar_handler)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

    def lidar_handler(self, data: PointCloud):
        float_array = Float64MultiArray()
        float_array.data = self.lidar_processor.processor(data.points)
        self.lidarPub.publish(float_array)


if __name__ == "__main__":
    new_lidar_detect = NewLidarObjectHandler()
    new_lidar_detect.listener()
