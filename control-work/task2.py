import open3d as o3d
import numpy as np

from task1 import draw_points, BezierSpline


def run_second(points, indexes, grid):
    points = np.array(points)
    points_to_draw = []
    bezier = BezierSpline(False)
    for i in range(5, grid[0]):
        p = points[i:i+grid[0]]
        res = bezier.run(p)
        for j in range(0, len(res)):
            points_to_draw.append(res[j])
    for i in points_to_draw:
        print(i[2])
    draw_points(points_to_draw, points)


"""def draw_points(points, indexes, grid):
    # create lines between neighboring points
    input_points = o3d.geometry.PointCloud()
    input_points.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([input_points], "NURBS surface", 800, 800)"""
