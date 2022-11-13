import random

import open3d as o3d
from numpy import arange


def run_first(points):
    for point in points:
        point.append(0)
    print(points)
    #draw_points(points)
    p = points
    for t in arange(0.0, 1, 0.01):
        p.append([(1-t)**3 * points[0][0] + 3 * (1 - t)**2*t*points[1][0] + 3*(1-t)*t**2*points[2][0] + t**3*points[3][0], (1-t)**3 * points[0][1] + 3 * (1 - t)**2*t*points[1][1] + 3*(1-t)*t**2*points[2][1] + t**3*points[3][1], 0])
    draw_points(p)


def draw_points(points):
    # colors = [[random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)] for _ in range(len(points))]
    colors = [[1, 0, 0] for _ in range(len(points))]
    for i in range(0, 10):
        colors[i] = [0, 0, 0]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd], "Bezier spline", 800, 800)
