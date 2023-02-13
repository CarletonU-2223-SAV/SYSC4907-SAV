import math
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import MapSerializer as MapSerializer
from Models import Point as PointModel, RoadSegmentType
from PIL import Image, ImageDraw, ImageColor

MAX_STEERING = 1.0
NH = 'NH'
CITY = 'CITY'

BACKGROUNDS = {
    NH: Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'scripts' / 'AirSim_maps' / 'NH_Top.png',
    CITY: Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'scripts' / 'AirSim_maps' / 'City_Top.png',
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

Point = Tuple[float, float]

metrics = []
errors = []


def analyze_time(times: List[str]):
    delta = (datetime.now() - datetime.strptime(times[0], '%Y-%m-%d %H:%M:%S')).days
    if delta > 7:
        # Give warning if test log is over a week old
        errors.append(f'Test log is old ({delta} days)')


def analyze_steering(val: str):
    # TODO Implement check
    if abs(float(val)) > MAX_STEERING:
        raise Exception('Steering exceeded maximum safe value.')


def analyze_collisions(data: Dict[str, any]):
    for i, collision_detected in enumerate(data[COLLISIONS]):
        if collision_detected:
            start_time = data[DATE][0]
            collision_time = data[DATE][i]
            delta = collision_time - start_time
            position = f'({data[POS_AIRSIM][i][0]:.2f}, {data[POS_AIRSIM][i][1]:.2f})'
            errors.append(f'Collision detected at {data[DATE][i]},'
                          f' position {position} (after {delta.seconds} seconds).')
            return


def analyze_path(actual: List[Point], expected: List[Point]):
    if not actual or not expected:
        raise Exception('Could not find points to analyze')

    for point in actual:
        line_1, line_2 = find_nearest_points(expected, point)
        (line_1x, line_1y), (line_2x, line_2y) = (line_1, line_2)
        point_x, point_y = point

        line_delta_x = line_2x - line_1x
        line_delta_y = line_2y - line_1y
        line_slope = line_delta_y / line_delta_x

        distance = (abs(line_delta_x * (line_1y - point_y) - (line_1x - point_x) * line_delta_y)
                    / distance_between_points(line_1, line_2))

        intersection_slope = -1 / line_slope
        delta_x = distance * math.cos(intersection_slope)
        delta_y = distance * math.sin(intersection_slope)

        intersection_x = point_x - delta_x
        intersection_y = point_y - delta_y

        if ((line_1x < intersection_x < line_2x and line_1y < intersection_y < line_2y)
                or (line_2x < intersection_x < line_1x and line_2y < intersection_y < line_1y)):
            # Point is between lines, good
            print(f'point ({point}) is between ({line_1}) and ({line_2})')
        else:
            # Point is past the end of one point
            print(f'point ({point}) is NOT between ({line_1}) and ({line_2})')


def find_nearest_points(path: List[Point], point: Point) -> Tuple[Point, Point]:
    # Find the nearest point
    min_distance = min(path, key=lambda crumb: distance_between_points(crumb, point))
    min_index = path.index(min_distance)

    if 0 < min_index < len(path):
        # Find nearest neighbour point
        neighbours = [path[min_index - 1], path[min_index + 1]]
        min_neighbour = min(neighbours, key=lambda crumb: distance_between_points(crumb, point))
    else:
        # Nearest neighbour point is only adjacent point
        index = 1 if min_index == 0 else (len(path) - 1)
        min_neighbour = path[index]
    return min_distance, min_neighbour


def distance_between_points(point_1: Tuple[float, float], point_2: Tuple[float, float]) -> float:
    point_1x, point_1y = point_1
    point_2x, point_2y = point_2
    return math.sqrt((point_1x - point_2x) ** 2 + (point_1y - point_2y) ** 2)


def analyze(test_case: str, pr_branch: Optional[str]):
    filepath = Path(__file__).parent / 'log' / f'{test_case}.txt'
    pickle = Path(__file__).parents[2] / 'src' / 'mapping_navigation' / 'paths' / f'{test_case}.pickle'
    pickle_map = MapSerializer.load_from_filename(pickle.__str__())

    # Store data. Need Airsim and GUI coords for image generation and path analysis
    data = {
        DATE: [],
        POS_AIRSIM: [],
        POS_GUI: [],
        STEERING: 0,
        THROTTLE: 0,
        COLLISIONS: [],
    }

    with open(filepath, 'r') as f:
        env = f.readline().strip()
        for line in f:
            line = line.strip()
            time, pos, steering, throttle, collision = line.split('|')
            x, y = [float(z) for z in pos.split(',')]
            line_pt = PointModel(x, y, RoadSegmentType.STRAIGHT)
            data[DATE].append(datetime.strptime(time, '%Y-%m-%d %H:%M:%S'))
            data[POS_GUI].append(line_pt.point_to_gui_coords(ENV_IDS[env]))
            data[POS_AIRSIM].append((x, y))
            data[COLLISIONS].append(collision == 'True')

    pickle_path = [(x, y) for x, y, _ in pickle_map.convert_path(0)]
    analyze_time(data[DATE])
    analyze_path(data[POS_AIRSIM], pickle_path)
    analyze_collisions(data)

    if pr_branch:
        # Running on GitHub for a PR, log metrics to a file
        pr_message_path = Path(__file__).parents[2] / 'pr_message.txt'
        with open(pr_message_path, 'w') as f:
            branch_name, commit_hash = pr_branch.split(',')
            f.write(f'### Analysis for commit {commit_hash[:7]} on branch {branch_name}\n')
            for metric in metrics:
                f.write(f'- {metric}\n')
            f.write('\n')
            if errors:
                f.write('**ERRORS:**\n')
                for error in errors:
                    f.write(f'- {error}\n')
    else:
        # Draw paths if script is manually run
        draw_path(data[POS_GUI], pickle_map.paths[0].get_gui_coords(), env, f'{test_case}.png')


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
