import pickle

import rospy
from pathlib import Path
from datetime import datetime
from typing import List
from common.bridge import get_bridge, Vector


class LogEntry:
    def __init__(self, time: datetime, pos: Vector, steering: float, throttle: float, has_collided: bool):
        self.time = time
        self.pos = pos
        self.steering = steering
        self.throttle = throttle
        self.has_collided = has_collided


class Logging:
    def __init__(self):
        self.client = get_bridge()
        rospy.init_node('logging', anonymous=True)
        rospy.on_shutdown(self.write_log)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.log: List[LogEntry] = []

    def create_log(self):
        while not rospy.is_shutdown():
            time = datetime.now()
            pos = self.client.get_position()
            steering = self.client.get_steering()
            throttle = self.client.get_throttle()
            has_collided = self.client.has_collided()
            self.log.append(LogEntry(time, pos, steering, throttle, has_collided))
            self.rate.sleep()

    def write_log(self):
        path_file: str = rospy.get_param('/path_file')
        path_name = path_file.split('.')[0]  # Get file name
        log_path = Path(__file__).parents[3] / 'testing' / 'AirSimTests' / 'log' / f'{path_name}.log'
        with open(log_path, 'w') as f:
            pickle.dump(self.log, f)


if __name__ == '__main__':
    logger = Logging()
    logger.create_log()
