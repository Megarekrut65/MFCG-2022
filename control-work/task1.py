import random

import open3d as o3d


def run_first(points):
    for point in points:
        point.append(0)
    print(points)
    draw_points(points)


def draw_points(points):
    colors = [[random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)] for _ in range(len(points))]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd], "Bezier spline", 800, 800)
