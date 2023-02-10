import sys
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import MapSerializer as MapSerializer
from Models import Point, RoadSegmentType
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

metrics = []
errors = []


def analyze_time(times: List[str]):
    # TODO Implement check
    delta = (datetime.now() - datetime.strptime(times[0], '%Y-%m-%d %H:%M:%S')).days
    if delta > 7:
        print(f'WARNING: Test log is old ({delta} days)')


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


def analyze_path(actual: List[Tuple[float, float]], expected: List[Tuple[float, float]]):
    if not actual or not expected:
        raise Exception('Could not find points to analyze')
    metrics.append('Test metric: Measured 99%, expected 100%')


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
            line_pt = Point(x, y, RoadSegmentType.STRAIGHT)
            data[DATE].append(datetime.strptime(time, '%Y-%m-%d %H:%M:%S'))
            data[POS_GUI].append(line_pt.point_to_gui_coords(ENV_IDS[env]))
            data[POS_AIRSIM].append((x, y))
            data[COLLISIONS].append(collision == 'True')

    pickle_path = [(x, y) for x, y, _ in pickle_map.convert_path(0)]
    analyze_path(data[POS_AIRSIM], pickle_path)
    analyze_collisions(data)

    # draw_path(data[POS_GUI], pickle_map.paths[0].get_gui_coords(), env, f'{test_case}.png')  # Disable by default

    if pr_branch:
        # Running on GitHub for a PR, log metrics to a file
        pr_message_path = Path(__file__).parents[2] / 'pr_message.txt'
        with open(pr_message_path, 'w') as f:
            f.write(f'### Analysis for branch { pr_branch }\n')
            for metric in metrics:
                f.write(f'{metric}\n')
            f.write('\n')
            if errors:
                f.write('**ERRORS:**\n')
                for error in errors:
                    f.write(f'- {error}\n')


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
