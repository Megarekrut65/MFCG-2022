from math import sqrt

import numpy as np
import open3d as o3d

from task1 import BezierSpline

def point_abs(a):
    return sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def build_t_points(points):
    n = len(points) - 1
    res = np.zeros(n + 1)
    res[0] = 0
    res[n] = 1
    d = 0
    for p in range(1, n+1):
        d += sqrt(point_abs(points[p] - points[p-1]))
    for p in range(1, n):
        res[p] = res[p-1] + sqrt(point_abs(points[p] - points[p-1]))/d
    return res

def build_knor(t_points, k):
    n = len(t_points) - 1
    res = np.zeros(n + 1)
    res[0:k+1] = 0
    res[n-k:n+1] = 1
    for j in range(k+1, n-k):
        res[j] = 0
        for i in range(j, j + k - 1):
            res[j] += t_points[i]
        res[j] /= k
    return res

def run_second(points, indexes, grid):
    print(build_t_points(np.array(points)))