
import json

from task1 import FirstTask


def first(first_task, number):
    f = open(f'points/{number}.json')
    data = json.load(f)
    first_task.run(points=data['curve'])


def first_all(first_task):
    for i in range(1, 25):
        print(i)
        first(first_task, i)


def main():
    ft = FirstTask(False)
    first(ft, 19)
    # first_all(ft)


if __name__ == '__main__':
    main()
