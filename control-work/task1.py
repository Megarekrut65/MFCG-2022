import numpy as np
import open3d as o3d
from numpy import arange


class FirstTask:
    def __init__(self, connect):
        self.connect = connect

    def run(self, points):
        if self.connect:
            points.append(points[0])

        np_points = np.array(points)
        n = len(np_points) - 1  # spline count

        control_points = self.find_control_points(np_points, n)
        # create points for drawing
        points_to_draw = []
        for i in range(0, n):
            for t in arange(0.0, 1, 0.01):
                points_to_draw.append(self.bezier_item(t, np_points[i],  # pi
                                                       control_points[2 * i],  # ai
                                                       control_points[2 * i + 1],  # bi
                                                       np_points[i + 1]))  # pi+1
        self.draw_points(points_to_draw, points)

    def find_control_points(self, np_points, n):
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

        if self.connect:  # boundary conditions for a closed spline
            matrix[0, 0] = 2
            matrix[0, 1] = -1
            matrix[0, 2 * n - 2] = 1
            matrix[0, 2 * n - 1] = -2

            matrix[2 * n - 1, 0] = 1
            matrix[2 * n - 1, 2 * n - 1] = 1

            vector[0] = [0, 0]
            vector[2 * n - 1] = [2 * np_points[0][0], 2 * np_points[0][1]]
        else:  # boundary conditions for an open spline
            matrix[0, 0] = 2
            matrix[0, 1] = -1
            matrix[2 * n - 1, 2 * n - 2] = -1
            matrix[2 * n - 1, 2 * n - 1] = 2

            vector[0] = np_points[0]
            vector[2 * n - 1] = np_points[n]

        return np.linalg.solve(matrix, vector)

    def bezier_item(self, t, p0, a0, b0, p1):
        return [(1 - t) ** 3 * p0[0] + 3 * (1 - t) ** 2 * t * a0[0] + 3 * (1 - t) * t ** 2 * b0[0] + t ** 3 * p1[0],
                (1 - t) ** 3 * p0[1] + 3 * (1 - t) ** 2 * t * a0[1] + 3 * (1 - t) * t ** 2 * b0[1] + t ** 3 * p1[1], 0]

    def draw_points(self, spline_points, points):
        # create lines between neighboring points
        lines = []
        for i in range(0, len(spline_points) - 1):
            lines.append([i, i + 1])
        spline_lines = o3d.geometry.LineSet()
        spline_lines.points = o3d.utility.Vector3dVector(spline_points)
        spline_lines.lines = o3d.utility.Vector2iVector(lines)

        points = [[points[i][0], points[i][1], 0] for i in range(0, len(points))]  # add z coordinate to points
        input_points = o3d.geometry.PointCloud()
        input_points.points = o3d.utility.Vector3dVector(points)
        input_points.colors = o3d.utility.Vector3dVector([[1, 0, 0] for i in range(0, len(points))])

        o3d.visualization.draw_geometries([spline_lines, input_points], "Bezier spline", 800, 800)
