from math import sqrt

import numpy as np
import open3d as o3d

from task1 import BezierSpline, draw_points


def point_abs(a):
    res = 0
    for i in range(0, len(a)):
        res += a[i] ** 2
    return sqrt(res)


class NURBSSpline:
    def __init__(self, points, k, t_points=None):
        self.points = np.array(points)
        self.n = len(points) - 1
        self.k = k
        self.p = k - 1
        self.m = self.n + self.k + 1
        self.t_points = t_points
        if t_points is None:
            self.t_points = self.build_t_points()
        self.knot = self.build_knot()
        self.w = np.full(self.n + 1, 1)
        # create coefficient matrix and vector b
        matrix = np.zeros((self.n + 1, self.n + 1), np.float)
        vector = self.points
        matrix[0] = np.zeros(self.n + 1)
        matrix[0][0] = 1
        matrix[self.n] = np.zeros(self.n + 1)
        matrix[self.n][self.n] = 1
        for i in range(1, self.n):
            sum_n = 0
            for j in range(0, self.n + 1):
                matrix[i][j] = self.basis_function(j, self.p, self.t_points[i]) * self.w[j]
                sum_n += matrix[i][j]
            matrix[i] /= sum_n
        self.x = np.linalg.solve(matrix, vector)

    def eval(self, t):
        if t == 0:
            return self.points[0]
        if t == 1:
            return self.points[self.n]
        res = 0
        sum_n = 0
        for i in range(0, self.n + 1):
            item = self.basis_function(i, self.p, t) * self.w[i]
            sum_n += item
            res += item * self.x[i]
        return res / sum_n

    def build_t_points(self):
        t_points = np.zeros(self.n + 1)
        t_points[0] = 0
        t_points[self.n] = 1
        d = 0
        for p in range(1, self.n + 1):
            d += sqrt(point_abs(self.points[p] - self.points[p - 1]))
        for p in range(1, self.n):
            t_points[p] = t_points[p - 1] + sqrt(point_abs(self.points[p] - self.points[p - 1])) / d
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


def run_second_as_first(points, k):
    nurbs = NURBSSpline(points, k)
    points_to_draw = []
    for t in np.arange(0.0, 1.01, 0.01):
        points_to_draw.append(nurbs.eval(t))
    draw_points(points_to_draw, points)


def get_points_in_row(points, row, size):
    return [points[i * size + row] for i in range(0, size)]


def get_points_in_column(points, col, size):
    return [points[col * size + i] for i in range(0, size)]


class NURBSSurface:
    def __init__(self, points, grid, k1, k2):
        self.points = np.array(points)
        self.n1 = grid[0] - 1
        self.n2 = grid[1] - 1
        self.k1 = k1
        self.p1 = k1 - 1
        self.k2 = k2
        self.p2 = k2 - 1
        self.m1 = self.n1 + self.k1 + 1
        self.m2 = self.n2 + self.k2 + 1
        self.t1_points = self.build_t1_points()
        self.t2_points = self.build_t2_points()
        self.knot1 = self.build_knot(self.t1_points, self.k2, self.p2, self.m2, self.n2)
        self.knot2 = self.build_knot(self.t2_points, self.k1, self.p1, self.m1, self.n1)
        self.w = np.full((self.n1 + 1, self.n2 + 1), 1)
        self.r = [[] for _ in range(0, self.n2 + 1)]
        for t in range(0, self.n1 + 1):
            matrix = np.zeros((self.n2 + 1, self.n2 + 1))
            vector = get_points_in_column(self.points, t, self.n2 + 1)
            matrix[0] = np.zeros(self.n1 + 1)
            matrix[0][0] = 1
            matrix[self.n1] = np.zeros(self.n1 + 1)
            matrix[self.n1][self.n1] = 1
            for i in range(1, self.n2):
                for j in range(0, self.n2 + 1):
                    matrix[i][j] = self.basis_function(j, self.p2, self.t1_points[i], self.knot1)
            x = np.linalg.solve(matrix, vector)
            for i in range(0, len(x)):
                self.r[i].append(x[i])
        self.p = []
        for t in range(0, self.n1 + 1):
            matrix = np.zeros((self.n1 + 1, self.n1 + 1), np.float)
            vector = self.r[t]
            matrix[0] = np.zeros(self.n2 + 1)
            matrix[0][0] = 1
            matrix[self.n2] = np.zeros(self.n2 + 1)
            matrix[self.n2][self.n2] = 1
            for i in range(1, self.n1):
                for j in range(0, self.n1 + 1):
                    matrix[i][j] = self.basis_function(j, self.p1, self.t2_points[i], self.knot2)
            x = np.linalg.solve(matrix, vector)
            self.p.append(x)

    def basis_function(self, i, p, t, knot):
        if p == 0:
            return 1 if t == knot[i] or (t > knot[i] and t < knot[i+1]) else 0
        res = 0
        if knot[i + p] != knot[i]:
            res += (t - knot[i]) / (knot[i + p] - knot[i]) * self.basis_function(i, p - 1, t, knot)
        if knot[i + p + 1] != knot[i + 1]:
            res += (knot[i + p + 1] - t) / (knot[i + p + 1] - knot[i + 1]) * self.basis_function(i + 1,
                                                                                                 p - 1,
                                                                                                 t, knot)
        return res

    def build_knot(self, t_points, k, p, m, n):
        knot = np.zeros(m)
        knot[0:k] = 0
        knot[m - k:m] = 1
        for j in range(1, n - p + 1):
            knot[j + p] = 0
            for i in range(j, j + p):
                knot[j + p] += t_points[i]
            knot[j + p] /= p
        return knot

    def eval(self, t1, t2):
        res = 0
        for i in range(0, self.n1 + 1):
            for j in range(0, self.n2 + 1):
                res += self.basis_function(i, self.p1, t1, self.knot1) * self.basis_function(j, self.p2,
                                                                                             t2, self.knot2) * \
                       self.p[i][j]
        return res

    def build_t1_points(self):
        (_, d) = self.points.shape
        t1_points = np.zeros(self.n1 + 1)
        t_matrix = [[] for _ in range(0, self.n2 + 1)]
        for k in range(0, self.n1 + 1):
            points = get_points_in_row(self.points, k, self.n2 + 1)
            t_points = self.build_t_points(self.n2, points)
            for i in range(0, self.n2 + 1):
                t_matrix[i].append(t_points[i])
        for i in range(0, self.n1 + 1):
            for j in range(0, self.n2 + 1):
                t1_points[i] += t_matrix[i][j]
            t1_points[i] /= (self.n2 + 1)
        return t1_points

    def build_t2_points(self):
        (_, d) = self.points.shape
        t2_points = np.zeros(self.n2 + 1)
        t_matrix = [[] for _ in range(0, self.n1 + 1)]
        for k in range(0, self.n2 + 1):
            points = get_points_in_column(self.points, k, self.n1 + 1)
            t_points = self.build_t_points(self.n1, points)
            for i in range(0, self.n1 + 1):
                t_matrix[i].append(t_points[i])
        for i in range(0, self.n2 + 1):
            for j in range(0, self.n1 + 1):
                t2_points[i] += t_matrix[i][j]
            t2_points[i] /= (self.n1 + 1)
        return t2_points

    def build_t_points(self, n, points):
        t_points = np.zeros(n + 1)
        t_points[0] = 0
        t_points[n] = 1
        d = 0
        for p in range(1, n + 1):
            d += point_abs(points[p] - points[p - 1])
        for p in range(1, n):
            t_points[p] = t_points[p - 1] + point_abs(points[p] - points[p - 1]) / d
        return t_points


def run_second(points, indices, grid_size, k):
    nurbs = NURBSSurface(points, grid_size, 4, 4)
    points_to_draw = []
    for t1 in np.arange(0.0, 1.0, 0.01):
        for t2 in np.arange(0.0, 1.0, 0.01):
            points_to_draw.append(nurbs.eval(t1,t2))

    draw_points(points, points_to_draw)
