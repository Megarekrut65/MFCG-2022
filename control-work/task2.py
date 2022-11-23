from math import sqrt

import numpy as np
import open3d as o3d

from task1 import BezierSpline, draw_points


def point_abs(a):
    res = 0
    for i in range(0, len(a)):
        res += a[i] ** 2
    return sqrt(res)


class NURBS:
    def __init__(self, points, k):
        self.points = np.array(points)
        print(f"points: {self.points}")
        self.n = len(points) - 1
        self.k = k
        self.p = k - 1
        self.m = self.n + self.k + 1
        self.t_points = self.build_t_points()
        print(f"t~: {self.t_points}")
        self.knot = self.build_knot()
        print(f"knot: {self.knot}")
        print(f"{self.basis_function(2, 0, 0)}")

        (_, dimension) = self.points.shape
        # create coefficient matrix and vector b
        matrix = np.zeros((self.n + 1, self.n + 1), np.float)
        vector = self.points
        matrix[0] = np.zeros(self.n + 1)
        matrix[0][0] = 1
        matrix[self.n] = np.zeros(self.n + 1)
        matrix[self.n][self.n] = 1
        for i in range(1, self.n):
            for j in range(0, self.n + 1):
                matrix[i][j] = self.basis_function(j, self.p, self.t_points[i])
        print(f"matrix: {matrix}")
        print(f"vector: {vector}")
        self.x = np.linalg.solve(matrix, vector)
        print(f"x: {self.x}")

    def eval(self, t):
        if t == 0:
            return self.points[0]
        if t == 1:
            return self.points[self.n]
        res = 0
        for i in range(0, self.n + 1):
            res += self.basis_function(i, self.p, t) * self.x[i]
        return res

    def build_t_points(self):
        t_points = np.zeros(self.n + 1)
        t_points[0] = 0
        t_points[self.n] = 1
        d = 0
        for p in range(1, self.n + 1):
            d += point_abs(self.points[p] - self.points[p - 1])
        for p in range(1, self.n):
            t_points[p] = t_points[p - 1] + point_abs(self.points[p] - self.points[p - 1]) / d
        return t_points

    def build_knot(self):
        knot = np.zeros(self.m)
        knot[0:self.k] = 0
        knot[self.m - self.k:self.m] = 1
        for j in range(1, self.n - self.p + 1):
            knot[j + self.p] = 0
            for i in range(j, j + self.p):
                knot[j + self.p] += self.t_points[i]
            knot[j + self.p] /= self.p
        return knot

    def basis_function(self, i, p, t):
        if p == 0:
            return 1 if t == self.knot[i] or (t > self.knot[i] and t < self.knot[i + 1]) else 0
        res = 0
        if self.knot[i + p] != self.knot[i]:
            res += (t - self.knot[i]) / (self.knot[i + p] - self.knot[i]) * self.basis_function(i, p - 1, t)
        if self.knot[i + p + 1] != self.knot[i + 1]:
            res += (self.knot[i + p + 1] - t) / (self.knot[i + p + 1] - self.knot[i + 1]) * self.basis_function(i + 1,
                                                                                                                p - 1,
                                                                                                                t)
        return res


def run_second(nurbs):
    points_to_draw = []
    for t in np.arange(0.0, 1.01, 0.01):
        points_to_draw.append(nurbs.eval(t))
    draw_points(points_to_draw, nurbs.points)
