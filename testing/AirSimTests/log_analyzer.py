import sys
from datetime import datetime
from pathlib import Path


def analyze(test_case: str):
    filepath = Path(__file__).parent / 'log' / f'{test_case}.txt'
    with open(filepath, 'r') as f:
        date = f.readline().rstrip()
        delta = (datetime.now() - datetime.strptime(date, '%Y-%m-%d %H:%M:%S')).days
        if delta > 7:
            print(f'WARNING: Test log is old ({delta} days)')

        # further analysis once test logs have been gathered


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('Test config error. Incorrect arguments supplied to log_analyzer.py.')

    case = sys.argv[1]
    analyze(case)
