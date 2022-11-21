import rospy
import airsim
from pathlib import Path
from datetime import datetime


class Logging:
    def __init__(self):
        host_ip = rospy.get_param('/host_ip')
        self.client = airsim.CarClient(ip=host_ip)
        self.client.confirmConnection()

        rospy.init_node('logging', anonymous=True)
        rospy.on_shutdown(self.write_log)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.max_steering = 0
        self.max_acc = 0

    def log(self):
        while not rospy.is_shutdown():
            car_controls = self.client.getCarControls()
            self.max_steering = max(car_controls.steering, self.max_steering, key=abs)
            self.max_acc = max(car_controls.throttle, self.max_acc, key=abs)
            self.rate.sleep()

    def write_log(self):
        car_state = self.client.getCarState()
        collisions = self.client.simGetCollisionInfo()

        path_file: str = rospy.get_param('/path_file')
        path_name = path_file.split('.')[0]  # Get file name
        log_path = Path(__file__).parents[3] / 'testing' / 'AirSimTests' / 'log' / f'{path_name}.txt'

        with open(log_path, 'w') as f:
            f.write(f'{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
            f.write(f'steering {self.max_steering}\n')
            f.write(f'acceleration {self.max_acc}\n')
            f.write(f'collisions {collisions.has_collided}\n')


if __name__ == '__main__':
    logger = Logging()
    logger.log()
