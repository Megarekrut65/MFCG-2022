from task1 import run_first
import json


def main():
    for i in range(1, 25):
        f = open(f'points/{i}.json')
        data = json.load(f)
        print(i)
        run_first(points=data['curve'])


if __name__ == '__main__':
    main()
