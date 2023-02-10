from abc import ABC
import numpy as np
from typing import List

_instance = None

SCENE_IMAGE = 0
DEPTH_IMAGE = 3
SEGMENTATION_IMAGE = 5


class CarControls:
    def __init__(self, throttle=0.0, steering=0.0, brake=0.0):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake


class Image:
    image_data_uint8 = np.uint8(0)
    image_data_float = 0.0
    width = 0
    height = 0


class Vector:
    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val


class Quaternion:
    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
        self.w_val = w_val


class Bridge(ABC):
    def get_image(self, camera_id: int, image_type: int, pixels_as_float=False) -> Image:
        pass

    def write_image(self, filepath: str, img: np.ndarray) -> None:
        pass

    def set_controls(self, controls: CarControls) -> None:
        pass

    def get_steering(self) -> float:
        pass

    def get_throttle(self) -> float:
        pass

    def get_position(self) -> Vector:
        pass

    def get_orientation(self) -> Quaternion:
        pass

    def create_binvox(self, position: Vector, x: int, y: int, z: int, res: float, of: str) -> None:
        pass

    def get_lidar_point_cloud(self) -> List[float]:
        pass

    def get_speed(self) -> float:
        pass

    def has_collided(self) -> bool:
        pass


def get_bridge() -> Bridge:
    global _instance
    if _instance is None:
        from common.airsim_bridge import AirSimBridge
        _instance = AirSimBridge()
    return _instance
