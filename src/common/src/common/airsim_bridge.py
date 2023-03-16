import airsim
import rospy
from common.bridge import Bridge, CarControls, Vector
from numpy import ndarray


class AirSimBridge(Bridge):
    """
    Create a bridge to simplify mocking when testing, switching to different sim
    """
    def __init__(self):
        host_ip = rospy.get_param('/host_ip')

        self.client = airsim.CarClient(ip=host_ip)
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def get_image(self, camera_id: int, image_type: int, pixels_as_float=False, compress=False):
        return self.client.simGetImages(
            [airsim.ImageRequest(camera_id, image_type, pixels_as_float, compress)])[0]

    def write_image(self, filepath: str, img: ndarray):
        airsim.write_png(filepath, img)

    def set_controls(self, controls: CarControls):
        airsim_controls = airsim.CarControls(controls.throttle, controls.steering, controls.brake)
        self.client.setCarControls(airsim_controls)

    def get_steering(self):
        return self.client.getCarControls().steering

    def get_throttle(self):
        return self.client.getCarControls().throttle

    def get_position(self):
        position = self.client.simGetGroundTruthKinematics("").position
        return Vector(position.x_val, position.y_val, position.z_val)

    def get_orientation(self):
        return self.client.getCarState().kinematics_estimated.orientation

    def create_binvox(self, position, x, y, z, res, of):
        airsim_pos = airsim.Vector3r(position.x_val, position.y_val, position.z_val)
        self.client.simCreateVoxelGrid(airsim_pos, x, y, z, res, of)

    def get_lidar_point_cloud(self):
        return self.client.getLidarData().point_cloud

    def get_speed(self):
        return self.client.getCarState().speed

    def has_collided(self):
        return self.client.simGetCollisionInfo().has_collided
