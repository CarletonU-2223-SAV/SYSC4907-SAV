import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray

from cluster_detection import ClusterDetection

# Size of the queue for object detection
q_size = 15


class LidarObjectDetect:

    def __init__(self):
        self.lidarPub = rospy.Publisher("lidar_data", Float64MultiArray, queue_size=q_size)
        self.cluster_detection = ClusterDetection()

    def listener(self):
        rospy.init_node("lidar_object_detect", anonymous=True)
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

    def handle_lidar_data(self, data: PointCloud):
        # Bounding boxes are sent as a flat array
        float_array = Float64MultiArray()
        float_array.data = self.cluster_detection.find_bounding_boxes(data.points)
        self.lidarPub.publish(float_array)


if __name__ == "__main__":
    # Do something
    lidar_detection = LidarObjectDetect()
    lidar_detection.listener()
