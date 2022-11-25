import json

from task1 import run_first
from task2 import run_second


def main():
    f = open(f'19.json')
    data = json.load(f)

    run_first(points=data['curve'], connect=False)
    run_first(points=data['curve'], connect=True)

    run_second(data["surface"]["points"], data["surface"]["indices"], data["surface"]["gridSize"], 4, 4)


if __name__ == '__main__':
    main()
