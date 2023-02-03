#!/usr/bin/env python3
import rospy
from pathlib import Path
from common.bridge import get_bridge, Vector

def binvox_map():
    rospy.init_node('binvox_gene', anonymous=True)
    # connect to the AirSim simulator
    sim = get_bridge()
    counter = 0
    while not rospy.is_shutdown():
        pos_curr = sim.get_position()
        x_curr = pos_curr.x_val
        y_curr = pos_curr.y_val
        z_curr = pos_curr.z_val
        center = Vector(x_curr, y_curr, z_curr)
        counter_str = str(counter)
        output_path = Path(__file__).parent / "binvox maps" / f"map{counter_str}.binvox"
        sim.create_binvox(center, 100, 100, 100, 0.5, str(output_path))
        counter = counter + 1


if __name__ == "__main__":
    binvox_map()
