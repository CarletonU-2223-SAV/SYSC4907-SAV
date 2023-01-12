#!/usr/bin/env python3
import airsim
import os
import rospy
import numpy as np
from pathlib import Path

def binvox_map():
    rospy.init_node('binvox_gene', anonymous=True)
    # connect to the AirSim simulator
    host_ip = rospy.get_param('/host_ip')
    client = airsim.CarClient(ip=host_ip)
    client.confirmConnection()
    counter = 0
    while not rospy.is_shutdown():
        pos_curr = client.getCarState().kinematics_estimated.position
        x_curr = pos_curr.x_val
        y_curr = pos_curr.y_val
        z_curr = pos_curr.z_val
        center = airsim.Vector3r(x_curr, y_curr, z_curr)
        counter_str = str(counter)
        output_path = Path(__file__).parent / "binvox maps" / f"map{counter_str}.binvox"
        client.simCreateVoxelGrid(center, 100, 100, 100, 0.5, str(output_path))
        counter = counter + 1


if __name__ == "__main__":
    binvox_map()
