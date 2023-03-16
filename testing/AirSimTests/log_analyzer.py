import json
import sys
from types import SimpleNamespace
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional
import MapSerializer as MapSerializer
from PIL import Image, ImageDraw, ImageColor
from sympy import Point, Segment
from common.models import LogEntry
from Models import X_COORD, Y_COORD, point_to_gui_coords

TARGET_AREA = 100.0
TARGET_TIME = 150
MAX_STEERING = 1.0
NH = 'NH'
CITY = 'CITY'

BACKGROUNDS = {
    NH: Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'AirSimMaps' / 'maps' / 'NH_Top.png',
    CITY: Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'AirSimMaps' / 'maps' / 'City_Top.png',
}

ENV_IDS = {
    NH: 0,
    CITY: 1,
}

PR_COMMENT_FLAG = '--pr-branch'


class LogAnalyzer:
    def __init__(self, test_case: str, pr_branch: Optional[str]):
        self.test_case = test_case
        self.pr_branch = pr_branch

        filepath = Path(__file__).parent / 'log' / f'{self.test_case}.log'
        env = self.test_case.split('_')[0]
        self.env_id = ENV_IDS[env]
        map_path = Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'paths' / f'{self.test_case}.pickle'
        map_model = MapSerializer.load_from_filename(str(map_path))

        with open(filepath, 'r') as f:
            self.log: List[LogEntry] = json.load(f, object_hook=lambda d: SimpleNamespace(**d))

        if not self.log:
            raise Exception('No data to analyze')

        pickle_path = [Point(x, y, evaluate=False) for x, y, _ in map_model.convert_path(0)]
        self.path_img = PathImage(map_model.paths[0].get_gui_coords(), env)

        self.segments = []
        for i in range(len(pickle_path) - 1):
            self.segments.append(Segment(pickle_path[i], pickle_path[i + 1]))

        self.metrics = []
        self.warnings = []
        self.warning_marks = 0
        self.incident_positions = []

    def analyze_timestamp(self):
        delta = (datetime.now() - datetime.strptime(self.log[0].time, "%Y-%m-%d %H:%M:%S")).days
        if delta > 7:
            # Give warning if test log is over a week old
            self.warnings.append(f'Test log is old ({delta} days)')

    def analyze_steering(self, entry: LogEntry):
        if abs(entry.steering) > MAX_STEERING:
            self.warning_marks += 1
            self.warnings.append(f'Steering exceeded target value (incident {self.warning_marks})')
            self.incident_positions.append(entry.pos)

    def analyze_point(self, current_point: Tuple[float, float], last_point: Tuple[float, float]):
        point_tuple = current_point

        # Get shortest distance to pre-computed segments
        distance = min([s.distance(point_tuple) for s in self.segments])

        # Estimate area as a rectangle bounded by distance between point and segment with last point
        return distance * (Point(point_tuple).distance(last_point))

    def analyze_speed(self, start_time: datetime, end_time: datetime):
        delta = (end_time - start_time).seconds
        self.metrics.append(f'Log runtime was {delta} seconds. Target value is {TARGET_TIME}')

    def analyze(self):
        last_point: Tuple[float, float] = self.log[0].pos
        start_time: Optional[datetime] = None
        end_time: Optional[datetime] = None
        area = 0.0
        for entry in self.log:
            if start_time is None and (entry.pos[X_COORD] - last_point[X_COORD] < 1
                                       or entry.pos[Y_COORD] - last_point[Y_COORD] < 1):
                start_time = datetime.strptime(entry.time, "%Y-%m-%d %H:%M:%S")

            point_tuple = entry.pos
            self.path_img.draw_actual_segment(
                point_to_gui_coords(last_point, self.env_id),
                point_to_gui_coords(point_tuple, self.env_id)
            )

            area += self.analyze_point(entry.pos, last_point)
            self.analyze_steering(entry)

            if entry.has_collided:
                end_time = datetime.strptime(entry.time, "%Y-%m-%d %H:%M:%S")
                self.path_img.draw_collision_point(point_tuple)
                self.warnings.append(f'Collision detected at {entry.time},'
                                     f' position {entry.pos} (after {(end_time - start_time).seconds} seconds).')
                break

            if (entry.pos[X_COORD] - self.log[-1].pos[X_COORD] < 1
                    and entry.pos[Y_COORD] - self.log[-1].pos[Y_COORD] < 1):
                # Reached end of path, log end time
                end_time = datetime.strptime(entry.time, "%Y-%m-%d %H:%M:%S")
                break

            end_time = datetime.strptime(entry.time, "%Y-%m-%d %H:%M:%S")
            last_point = entry.pos

        self.metrics.append(f'Area between target and actual path is {area:.2f}. Target value is {TARGET_AREA:.2f}')
        self.analyze_speed(start_time, end_time)
        self.path_img.save(f'{self.test_case}.png')

        if self.pr_branch:
            # Running on GitHub for a PR, log metrics to a file and upload image
            pr_message_path = Path(__file__).parents[2] / 'pr_message.txt'
            with open(pr_message_path, 'w') as f:
                branch_name, commit_hash = self.pr_branch.split(',')
                f.write(f'### Analysis for commit {commit_hash[:7]} on branch {branch_name}\n')
                for metric in self.metrics:
                    f.write(f'- {metric}\n')
                f.write('\n')
                if self.warnings:
                    f.write('**Warnings:**\n')
                    for warning in self.warnings:
                        f.write(f'- {warning}\n')

                image_path = Path(__file__).parent / 'img' / f'{self.test_case}.png'
                from images_upload_cli.upload import imgur_upload
                url = imgur_upload(image_path.read_bytes())
                f.write(f'![Path analysis]({url})')


class PathImage:
    EXPECTED_COLOUR = ImageColor.getrgb('red')
    ACTUAL_COLOUR = ImageColor.getrgb('blue')
    COLLISION_COLOUR = ImageColor.getrgb('red')
    INCIDENT_COLOUR = ImageColor.getrgb('blue')
    INCIDENT_TEXT = ImageColor.getrgb('white')
    INCIDENT_RADIUS = 5

    def __init__(self, expected: List[Tuple[float, float]], env: str):
        self.img = Image.open(BACKGROUNDS[env])
        self.draw = ImageDraw.Draw(self.img)

        last_coords = expected.pop(0)
        for current_coords in expected:
            # Overlay pickle line
            self.draw.line([last_coords, current_coords], self.EXPECTED_COLOUR, 3)
            last_coords = current_coords

    def save(self, save_path: str):
        self.img.save(f'img/{save_path}')

    def draw_actual_segment(self, start: Tuple[float, float], end: Tuple[float, float]):
        self.draw.line([tuple(start), tuple(end)], self.ACTUAL_COLOUR, 3)

    def draw_collision_point(self, point: Tuple[float, float]):
        self.draw.text(point, "X", self.COLLISION_COLOUR)

    def draw_incidents(self, incidents: List[Tuple[float, float]]):
        for i, (x, y) in enumerate(incidents):
            self.draw.ellipse(
                (x - self.INCIDENT_RADIUS, y - self.INCIDENT_RADIUS,
                 x + self.INCIDENT_RADIUS, y + self.INCIDENT_RADIUS),
                fill=self.INCIDENT_COLOUR
            )
            self.draw.text((x, y), str(i + 1), self.INCIDENT_TEXT)


if __name__ == '__main__':
    if len(sys.argv) not in [2, 3]:
        raise Exception('Test config error. Incorrect arguments supplied to log_analyzer.py.')

    args = sys.argv[1:]
    branch = None
    case = args[0]
    if len(args) > 1:
        # PR flag is set
        for arg in args:
            if PR_COMMENT_FLAG in arg:
                branch = arg.split('=')[1]
            else:
                case = arg

    LogAnalyzer(case, branch).analyze()
