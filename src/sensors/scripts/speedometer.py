#!/usr/bin/env python3

import airsim

import rospy
from std_msgs.msg import Float64

def speedometer():
    pub = rospy.Publisher('sensor/speed', Float64, queue_size=1)
    rospy.init_node('speedometer', anonymous=True)
    rate = rospy.Rate(10) # 30 Hz

    # connect to the AirSim simulator 
    host_ip = rospy.get_param('/host_ip')
    client = airsim.CarClient(ip=host_ip)
    client.confirmConnection()

    while not rospy.is_shutdown():
        # Get the speed of the car
        car_speed = client.getCarState().speed
        rospy.loginfo(car_speed)
        pub.publish(car_speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        speedometer()
    except rospy.ROSInterruptException:
        pass