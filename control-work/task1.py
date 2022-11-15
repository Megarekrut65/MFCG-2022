import numpy as np
import open3d as o3d
from numpy import arange


def run_first(points):
    np_points = np.array(points)
    n = len(np_points) - 1  # splain count

    control_points = find_control_points(np_points, n)
    # create array with splain and control points
    points_for_bezier = []
    for i in range(0, n):
        points_for_bezier.append(np_points[i])  # pi
        points_for_bezier.append(control_points[2 * i])  # ai
        points_for_bezier.append(control_points[2 * i + 1])  # bi
    points_for_bezier.append(np_points[n])  # pn
    # create points for drawing
    points_to_draw = []
    for i in range(0, len(points_for_bezier) // 3):
        for t in arange(0.0, 1, 0.01):
            points_to_draw.append(bezier_item(t, points_for_bezier[3 * i], points_for_bezier[3 * i + 1],
                                              points_for_bezier[3 * i + 2], points_for_bezier[3 * i + 3]))
    draw_points(points_to_draw, points)


def find_control_points(np_points, n):
    # create coefficient matrix and vector b
    matrix = np.zeros((2 * n, 2 * n), np.float)
    vector = np.zeros((2 * n, 2), np.float)
    for i in np.arange(1, 2 * n - 2, 2):
        matrix[i, i - 1] = 1
        matrix[i, i] = -2
        matrix[i, i + 1] = 2
        matrix[i, i + 2] = -1
        matrix[i + 1, i] = 1
        matrix[i + 1, i + 1] = 1

        vector[i] = [0, 0]
        vector[i + 1] = [2 * np_points[i // 2 + 1][0], 2 * np_points[i // 2 + 1][1]]

    matrix[0, 0] = 2
    matrix[0, 1] = -1
    matrix[2 * n - 1, 2 * n - 2] = -1
    matrix[2 * n - 1, 2 * n - 1] = 2

    vector[0] = np_points[0]
    vector[2 * n - 1] = np_points[n]

    return np.linalg.solve(matrix, vector)


def bezier_item(t, p0, a0, b0, p1):
    return [(1 - t) ** 3 * p0[0] + 3 * (1 - t) ** 2 * t * a0[0] + 3 * (1 - t) * t ** 2 * b0[0] + t ** 3 * p1[0],
            (1 - t) ** 3 * p0[1] + 3 * (1 - t) ** 2 * t * a0[1] + 3 * (1 - t) * t ** 2 * b0[1] + t ** 3 * p1[1], 0]


def draw_points(spline_points, points):
    points = [[points[i][0], points[i][1], 0] for i in range(0, len(points))]
    lines = []
    for i in range(0, len(spline_points) - 1):
        lines.append([i, i+1])
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(spline_points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector([[1, 0, 0] for i in range(0, len(points))])
    o3d.visualization.draw_geometries([line_set, pcd], "Bezier spline", 800, 800)
