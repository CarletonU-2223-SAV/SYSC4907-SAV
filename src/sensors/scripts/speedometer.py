#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from common.bridge import get_bridge

def speedometer():
    pub = rospy.Publisher('sensor/speed', Float64, queue_size=1)
    rospy.init_node('speedometer', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # connect to the AirSim simulator 
    sim = get_bridge()

    while not rospy.is_shutdown():
        # Get the speed of the car
        car_speed = sim.get_speed()
        rospy.loginfo(car_speed)
        pub.publish(car_speed)
        rate.sleep()


if __name__ == '__main__':
    try:
        speedometer()
    except rospy.ROSInterruptException:
        pass