#!/usr/bin/env python3
import time

from pid_controller import PIDController
from pid_controller import ThrottleAction
from cluster_detection import ClusterDetection

import rospy
from mapping_navigation.msg import PathData
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64, Float64MultiArray
import os

class CruiseControl:

    def __init__(self):
        self.steeringPub = rospy.Publisher("steering", Float64, queue_size = 10)
        self.brakingPub = rospy.Publisher("braking", Float64, queue_size = 10)
        self.throttlePub = rospy.Publisher("throttling", Float64, queue_size = 10)
        self.lidarPub = rospy.Publisher("lidar_data", Float64MultiArray, queue_size = 10)

        self.cluster_detection = ClusterDetection()

        self.pidController = PIDController()
        self.currentSpeed = 0.0
        self.lastTime = time.time()

        # Used for automated testing
        self.outputSpeedFile = open("speed_output.txt", "w")

    def listener(self):
        rospy.init_node("cruise_control", anonymous=True)
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)
        # Define Path data structure
        rospy.Subscriber("pathData", PathData, self.handle_path_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)
        rospy.Subscriber("target_speed", Float64, self.handle_new_target_speed)

        # Node is publisher and subscriber- cannot use spin; the publisher methods will never get called

        # Probably should have same rate for this node and speedometer to make them more in sync
        # Higher the rate the smoother the constant speed is, but will take more CPU power
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_results()
            rate.sleep()

    def handle_lidar_data(self, data: PointCloud):
        # Bounding boxes are sent as a flat array
        float_array = Float64MultiArray()
        float_array.data = self.cluster_detection.find_bounding_boxes(data.points)
        self.lidarPub.publish(float_array)

    def handle_path_data(self, data):
        print("Obtained path data")

    def handle_new_target_speed(self, new_target_speed: Float64):
        self.pidController.update_target_speed(new_target_speed.data)

    def handle_speed_data(self, speed: Float64):
        self.currentSpeed = speed.data

    def publish_results(self):

        delta_time = time.time() - self.lastTime

        if delta_time <= 0.0:
            return

        self.lastTime += delta_time
        (throttle_action, throttle_value) = self.pidController.update_pid_output(self.currentSpeed, delta_time)

        if throttle_action == ThrottleAction.Accelerate:
            self.throttlePub.publish(throttle_value)
            self.brakingPub.publish(0.0)
            rospy.loginfo("Setting throttle: {}".format(throttle_value))
        else:
            self.brakingPub.publish(throttle_value)
            self.throttlePub.publish(0.0)
            rospy.loginfo("Setting brakes: {}".format(throttle_value))

        self.outputSpeedFile.write("{},{}\n".format(delta_time, self.currentSpeed))


if __name__ == "__main__":
    # Do something
    cc = CruiseControl()
    cc.listener()
