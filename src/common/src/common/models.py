import json
from typing import Tuple, Any


class LogEntry:
    def __init__(self,
                 time: str,
                 pos: Tuple[float, float],
                 steering: float,
                 throttle: float,
                 has_collided: bool,
                 lidar_detections: int):
        self.time = time
        self.pos = pos
        self.steering = steering
        self.throttle = throttle
        self.has_collided = has_collided
        self.lidar_detections = lidar_detections

    def default(self, o: Any) -> Any:
        return json.dumps(dict(o))

