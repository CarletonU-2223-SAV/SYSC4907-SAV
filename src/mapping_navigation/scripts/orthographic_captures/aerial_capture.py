import airsim
from math import pi

# Instructions to capture aerial images of any AirSim Environment for TestGUI.py:
#  - The settings.json file seen in folder "orthographic_captures" must replace the
#    settings.json file used when loading AirSim to properly capture image.
#  - Launch AirSim environment.
#  - Open command prompt, navigate go to "mapping_navigation\scripts\orthographic_captures"
#    and execute "python aerial_capture.py".

# Side note: z axis in "airsim.vector3r(x,y,z)" may need to be tweaked depending on environment

client = airsim.VehicleClient()
client.confirmConnection()

orientation = airsim.to_quaternion(-pi / 2, 0, 0)
position = airsim.Vector3r(0, 0, -600)
pose = airsim.Pose(position, orientation)

client.simSetVehiclePose(pose, ignore_collision=True)
image = client.simGetImage('', airsim.ImageType.Scene)

airsim.write_file('NAME_HERE.png', image)
