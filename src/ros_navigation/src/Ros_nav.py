#!/usr/bin/env python3
import airsim
import os
import rospy


def binvox_map():
    rospy.init_node('binvox_gene', anonymous=True)

    # connect to the AirSim simulator
    host_ip = rospy.get_param('/host_ip')
    client = airsim.CarClient(ip=host_ip)
    client.confirmConnection()
    center = airsim.Vector3r(0, 0, 0)
    output_path = os.path.join(os.getcwd(), "map.binvox")
    client.simCreateVoxelGrid(center, 100, 100, 100, 0.5, output_path)


if __name__ == "__main__":
    binvox_map()
