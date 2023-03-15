import pickle
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional
import MapSerializer as MapSerializer
from Models import Point as PointModel, RoadSegmentType
from PIL import Image, ImageDraw, ImageColor
from sympy import Point2D, Point, Segment
from vehicle_logging import LogEntry

TARGET_AREA = 100.0
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

DATE = 'date'
POS_AIRSIM = 'pos_airsim'
POS_GUI = 'pos_gui'
STEERING = 'steering'
THROTTLE = 'throttle'
COLLISIONS = 'collided'

PR_COMMENT_FLAG = '--pr-branch'

metrics = []
warnings = []


def analyze_time(times: List[datetime]):
    delta = (datetime.now() - times[0]).days
    if delta > 7:
        # Give warning if test log is over a week old
        warnings.append(f'Test log is old ({delta} days)')


def analyze_steering(val: str):
    # TODO Implement check
    if abs(float(val)) > MAX_STEERING:
        raise Exception('Steering exceeded maximum safe value.')


def analyze_collisions(log: List[LogEntry]):
    for i, entry in enumerate(log):
        if entry.has_collided:
            start_time = log[0].time
            collision_time = entry.time
            delta = collision_time - start_time
            position = f'({log[i].pos.x_val:.2f}, {log[i].pos.y_val:.2f})'
            warnings.append(f'Collision detected at {collision_time},'
                            f' position {position} (after {delta.seconds} seconds).')
            return


def analyze_path(actual: List[Point2D], expected: List[Point2D]):
    if not actual or not expected:
        raise Exception('Could not find points to analyze')

    segments = []
    for i in range(len(expected) - 1):
        segments.append(Segment(expected[i], expected[i + 1]))

    area = 0.0
    for i, point in enumerate(actual[1:]):
        # Get shortest distance to pre-computed segments
        distance = min([s.distance(point) for s in segments])

        # Estimate area as a rectangle bounded by distance between point and segment with last point
        area += distance * (point.distance(actual[i]))
    metrics.append(f'Area between target and actual path is {area:.2f}. Target value is {TARGET_AREA:.2f}')


def analyze(test_case: str, pr_branch: Optional[str]):
    filepath = Path(__file__).parent / 'log' / f'{test_case}.log'
    env = str(filepath).split('_')[0]
    map_path = Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'paths' / f'{test_case}.pickle'
    map_model = MapSerializer.load_from_filename(str(map_path))

    with open(filepath, 'r') as f:
        log: List[LogEntry] = pickle.load(f)

    pickle_path = [Point(x, y, evaluate=False) for x, y, _ in map_model.convert_path(0)]
    analyze_time([entry.time for entry in log])
    analyze_path([Point(entry.pos.x_val, entry.pos.y_val) for entry in log], pickle_path)
    analyze_collisions(log)

    if pr_branch:
        # Running on GitHub for a PR, log metrics to a file
        pr_message_path = Path(__file__).parents[2] / 'pr_message.txt'
        with open(pr_message_path, 'w') as f:
            branch_name, commit_hash = pr_branch.split(',')
            f.write(f'### Analysis for commit {commit_hash[:7]} on branch {branch_name}\n')
            for metric in metrics:
                f.write(f'- {metric}\n')
            f.write('\n')
            if warnings:
                f.write('**Warnings:**\n')
                for warning in warnings:
                    f.write(f'- {warning}\n')
    else:
        # Draw paths if script is manually run
        points = [PointModel(entry.pos.x_val, entry.pos.y_val, RoadSegmentType.STRAIGHT) for entry in log]
        draw_path(
            [point.point_to_gui_coords(ENV_IDS[env]) for point in points],
            map_model.paths[0].get_gui_coords(),
            env,
            f'{test_case}.png'
        )


def draw_path(actual: List[Tuple[float, float]], expected: List[Tuple[float, float]], env: str, save_path: str):
    """
    Use this to create an image comparing the vehicle's actual path with its pickle file
    """
    paths = [(actual, ImageColor.getrgb('blue')), (expected, ImageColor.getrgb('red'))]
    with Image.open(BACKGROUNDS[env]) as img:
        draw = ImageDraw.Draw(img)
        for i, (path, colour) in enumerate(paths):
            last_coords = path.pop(0)
            for current_coords in path:
                # Overlay pickle line
                draw.line([last_coords, current_coords], colour, 3)
                last_coords = current_coords
        img.save(f'img/{save_path}')
    pass


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

    analyze(case, branch)
