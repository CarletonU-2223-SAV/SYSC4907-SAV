import airsim
import os
import rospy

host_ip = rospy.get_param('/host_ip')
client = airsim.CarClient(ip=host_ip)
client.confirmConnection()
center = airsim.Vector3r(0, 0, 0)
output_path = os.path.join(os.getcwd(), "map.binvox")
client.simCreateVoxelGrid(center, 100, 100, 100, 0.5, output_path)
