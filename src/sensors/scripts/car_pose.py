#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from common.bridge import get_bridge

def airpub():
    pub = rospy.Publisher("airsimPose", PoseStamped, queue_size=1)
    rospy.init_node('airpub', anonymous=True)
    rate = rospy.Rate(3)  # 3hz

    # connect to the AirSim simulator
    sim = get_bridge()

    # start = time.time()

    while not rospy.is_shutdown():
        # get state of the car
        pos = sim.get_position()
        orientation = sim.get_orientation()
        # milliseconds = (time.time() - start) * 1000

        # populate PoseStamped ros message
        simPose = PoseStamped()
        simPose.pose.position.x = pos.x_val
        simPose.pose.position.y = pos.y_val
        simPose.pose.position.z = pos.z_val
        simPose.pose.orientation.w = orientation.w_val
        simPose.pose.orientation.x = orientation.x_val
        simPose.pose.orientation.y = orientation.y_val
        simPose.pose.orientation.z = orientation.z_val
        simPose.header.stamp = rospy.Time.now()
        simPose.header.seq = 1
        simPose.header.frame_id = "simFrame"
        
        # log PoseStamped message
        rospy.loginfo(simPose)
        # publish PoseStamped message
        pub.publish(simPose)
        # sleeps until next cycle 
        rate.sleep()


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass

