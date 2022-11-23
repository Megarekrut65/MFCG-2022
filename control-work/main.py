import json

import numpy

from task1 import FirstTask
from task2 import NURBS, run_second


def first(first_task, number):
    f = open(f'points/{number}.json')
    data = json.load(f)
    first_task.run(points=data['curve'])


def first_all(first_task):
    for i in range(1, 25):
        print(i)
        first(first_task, i)


def second():
    f = open(f'points/{19}.json')
    data = json.load(f)
    nurbs = NURBS(data['curve'], 4)
    run_second(nurbs)
    # run_second(data["surface"]["points"], data["surface"]["indices"], data["surface"]["gridSize"])


def main():
    second()
    return
    ft = FirstTask(False)
    first(ft, 19)
    # first_all(ft)


if __name__ == '__main__':
    main()
