import rospy
from pathlib import Path
from datetime import datetime
from typing import List
from common.bridge import get_bridge


class Logging:
    def __init__(self):
        self.client = get_bridge()
        rospy.init_node('logging', anonymous=True)
        rospy.on_shutdown(self.write_log)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.data: List[List[str]] = []

    def log(self):
        while not rospy.is_shutdown():
            time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            pos = self.client.get_position()
            pos_str = ','.join([str(pos.x_val), str(pos.y_val)])
            steering = self.client.get_steering()
            throttle = self.client.get_throttle()
            collisions = self.client.has_collided()
            self.data.append([x.__str__() for x in [time, pos_str, steering, throttle, collisions]])
            self.rate.sleep()

    def write_log(self):
        path_file: str = rospy.get_param('/path_file')
        path_name = path_file.split('.')[0]  # Get file name
        log_path = Path(__file__).parents[3] / 'testing' / 'AirSimTests' / 'log' / f'{path_name}.txt'
        env = path_name.split('_')[0]
        with open(log_path, 'w') as f:
            f.write(f'{env}\n')
            for line in self.data:
                f.write('|'.join(line) + '\n')


if __name__ == '__main__':
    logger = Logging()
    logger.log()
