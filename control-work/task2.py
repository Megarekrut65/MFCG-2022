import open3d as o3d


def run_second(points, indexes, grid):
    draw_points(points, indexes, grid)


def draw_points(points, indexes, grid):
    # create lines between neighboring points
    input_points = o3d.geometry.PointCloud()
    input_points.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([input_points], "NURBS surface", 800, 800)
