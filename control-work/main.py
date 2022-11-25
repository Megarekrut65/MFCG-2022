import json

from task1 import run_first
from task2 import run_second


def main():
    f = open(f'19.json')
    data = json.load(f)

    run_first(data['curve'], connect=False)
    run_first(data['curve'], connect=True)

    surface = data["surface"]
    run_second(surface["points"], surface["indices"], surface["gridSize"], k1=4, k2=4, points_count=50)


if __name__ == '__main__':
    main()
