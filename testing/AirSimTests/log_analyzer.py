from datetime import datetime
from math import pow, sqrt
import sys

TEST_CASES = [
    'left_turn',
    'right_turn',
    'straight',
]


def analyze(test_case: str):
    with open(f'log/{test_case}.txt', 'r') as f:
        date = f.readline().rstrip()
        delta = (datetime.now() - datetime.strptime(date, '%Y-%m-%d %H:%M:%S')).days
        if delta > 7:
            raise Exception(f'Test log too old for case {test_case}')

        # some random test case to see
        data = f.readline().rstrip().split(',')
        length = sqrt(sum([pow(int(x), 2) for x in data]))
        if length != 5:
            raise Exception(f'Test case failed! Expected length 5, actual {length}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('Test config error. Incorrect arguments supplied to log_analyzer.py.')

    case = sys.argv[1]
    if case not in TEST_CASES:
        raise Exception('Test config error. Invalid test case supplied to log_analyzer.py.')

    analyze(case)
