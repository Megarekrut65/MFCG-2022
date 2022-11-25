from math import sqrt

import numpy as np
import open3d as o3d


def point_abs(a):
    res = 0
    for i in range(0, len(a)):
        res += a[i] ** 2
    return sqrt(res)


def get_points_in_row(points, row):
    return points[row]


def get_points_in_column(points, col):
    return [points[i][col] for i in range(0, len(points))]


def array_to_matrix(arr, indices, grid):
    matrix = np.zeros((grid[0], grid[1], 3), np.float)
    for i in range(0, len(arr)):
        ind = indices[i]
        matrix[ind[0]][ind[1]] = arr[i]
    return matrix


class NURBSSurface:
    def __init__(self, points_as_matrix, grid, k1, k2):
        self.points = points_as_matrix
        self.n1 = grid[0] - 1
        self.n2 = grid[1] - 1
        self.k1 = k1
        self.p1 = k1 - 1
        self.k2 = k2
        self.p2 = k2 - 1
        self.m1 = self.n1 + self.k1 + 1
        self.m2 = self.n2 + self.k2 + 1
        self.t1_points = self.__build_t1_points()
        self.t2_points = self.__build_t2_points()
        self.knot1 = self.__build_knot(self.t1_points, self.k2, self.p2, self.m2, self.n2)
        self.knot2 = self.__build_knot(self.t2_points, self.k1, self.p1, self.m1, self.n1)
        self.w = np.full((self.n1 + 1, self.n2 + 1), 1)
        self.p = self.__find_pij(self.__find_rij())

    def __find_rij(self):
        r = [[] for _ in range(0, self.n2 + 1)]
        for t in range(0, self.n1 + 1):
            matrix = np.zeros((self.n2 + 1, self.n2 + 1))
            vector = get_points_in_column(self.points, t)
            sum_w = 0
            for i in range(0, self.n2 + 1):
                for j in range(0, self.n2 + 1):
                    matrix[i][j] = self.__basis_function(j, self.p2, self.t1_points[i], self.knot1)
                    sum_w += matrix[i][j] * self.__basis_function(i, self.p1, self.t2_points[t], self.knot2) * self.w[i][j]
            x = np.linalg.solve(matrix/sum_w, vector)
            for i in range(0, len(x)):
                r[i].append(x[i])
        return r

    def __find_pij(self, rij):
        p = []
        for t in range(0, self.n1 + 1):
            matrix = np.zeros((self.n1 + 1, self.n1 + 1), np.float)
            vector = rij[t]
            for i in range(0, self.n1 + 1):
                for j in range(0, self.n1 + 1):
                    matrix[i][j] = self.__basis_function(j, self.p1, self.t2_points[i], self.knot2) * self.w[i][j]
            x = np.linalg.solve(matrix, vector)
            p.append(x)
        return p

    def __basis_function(self, i, p, t, knot):
        if p == 0:
            return 1 if t == knot[i] or (t > knot[i] and t < knot[i + 1]) or (t == 1 and knot[i + 1] == 1) else 0
        res = 0
        if knot[i + p] != knot[i]:
            res += (t - knot[i]) / (knot[i + p] - knot[i]) * self.__basis_function(i, p - 1, t, knot)
        if knot[i + p + 1] != knot[i + 1]:
            res += (knot[i + p + 1] - t) / (knot[i + p + 1] - knot[i + 1]) * self.__basis_function(i + 1,
                                                                                                   p - 1,
                                                                                                   t, knot)
        return res

    def __build_knot(self, t_points, k, p, m, n):
        knot = np.zeros(m)
        knot[0:k] = 0
        knot[m - k:m] = 1
        for j in range(1, n - p + 1):
            knot[j + p] = 0
            for i in range(j, j + p):
                knot[j + p] += t_points[i]
            knot[j + p] /= p
        return knot

    def __build_t1_points(self):
        (_, d) = self.points[0].shape
        t1_points = np.zeros(self.n1 + 1)
        t_matrix = [[] for _ in range(0, self.n2 + 1)]
        for k in range(0, self.n1 + 1):
            points = get_points_in_row(self.points, k)
            t_points = self.__build_t_points(self.n2, points)
            for i in range(0, self.n2 + 1):
                t_matrix[i].append(t_points[i])
        for i in range(0, self.n1 + 1):
            for j in range(0, self.n2 + 1):
                t1_points[i] += t_matrix[i][j]
            t1_points[i] /= (self.n2 + 1)
        return t1_points

    def __build_t2_points(self):
        (_, d) = self.points[0].shape
        t2_points = np.zeros(self.n2 + 1)
        t_matrix = [[] for _ in range(0, self.n1 + 1)]
        for k in range(0, self.n2 + 1):
            points = get_points_in_column(self.points, k)
            t_points = self.__build_t_points(self.n1, points)
            for i in range(0, self.n1 + 1):
                t_matrix[i].append(t_points[i])
        for i in range(0, self.n2 + 1):
            for j in range(0, self.n1 + 1):
                t2_points[i] += t_matrix[i][j]
            t2_points[i] /= (self.n1 + 1)
        return t2_points

    def __build_t_points(self, n, points):
        t_points = np.zeros(n + 1)
        t_points[0] = 0
        t_points[n] = 1
        d = 0
        for p in range(1, n + 1):
            d += point_abs(points[p] - points[p - 1])
        for p in range(1, n):
            t_points[p] = t_points[p - 1] + point_abs(points[p] - points[p - 1]) / d
        return t_points

    def eval(self, t1, t2):
        res = 0
        sum_w = 0
        for i in range(0, self.n1 + 1):
            for j in range(0, self.n2 + 1):
                item = self.__basis_function(i, self.p2, t1, self.knot1) * self.__basis_function(j, self.p1,
                                                                                                 t2, self.knot2) * self.w[i][j]
                sum_w += item
                res += item * self.p[i][j]
        return res/sum_w


def draw_surface(points_to_draw, points, row_size, col_size):
    # create mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points_to_draw)
    mesh.compute_vertex_normals()
    # create triangles for mesh
    triangles = []
    for i in range(0, row_size - 1):
        for j in range(0, col_size - 1):
            if ((j + i) % 2) == 0:
                triangles.append([i * row_size + j, i * row_size + j + 1, (i + 1) * row_size + j])
                triangles.append([i * row_size + j + 1, (i + 1) * row_size + j, (i + 1) * row_size + j + 1])
            else:
                triangles.append([i * row_size + j, i * row_size + j + 1, (i + 1) * row_size + j + 1])
                triangles.append([i * row_size + j, (i + 1) * row_size + j, (i + 1) * row_size + j + 1])
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(triangles))
    mesh.compute_triangle_normals()
    mesh.paint_uniform_color([0.5, 0.5, 0.5])

    input_points = o3d.geometry.PointCloud()
    input_points.points = o3d.utility.Vector3dVector(points)
    input_points.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(0, len(points))])

    o3d.visualization.draw_geometries([input_points, mesh], "NURBS surface", 800, 800, mesh_show_wireframe=True,
                                      mesh_show_back_face=True)


def run_second(points, indices, grid, k1, k2, points_count=20):
    nurbs = NURBSSurface(array_to_matrix(points, indices, grid), grid, k1, k2)
    points_to_draw = []
    t = np.linspace(0, 1, points_count)
    for _, t1 in enumerate(t):
        for _, t2 in enumerate(t):
            points_to_draw.append(nurbs.eval(t1, t2))
    draw_surface(points_to_draw, points, len(t), len(t))
