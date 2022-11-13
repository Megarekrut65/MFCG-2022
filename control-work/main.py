from task1 import run_first
import json


def main():
    f = open('19.json')
    data = json.load(f)
    run_first(points=data['curve'])


if __name__ == '__main__':
    main()
