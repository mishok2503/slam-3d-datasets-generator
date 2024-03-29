import sys
import time
from math import *
import open3d
import numpy as np
import json


def rot_m(rotation):
    a, b, c = rotation
    return np.array([
        [cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c), sin(a) * sin(b)],
        [sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b)],
        [sin(b) * sin(c), sin(b) * cos(c), cos(b)]
    ])


points = []
pos = np.array([0., 0., 0.])
rot = np.array([0., 0., 0.])
k = rot_m(rot)
with open(sys.argv[1], "r") as file:
    data = json.load(file)["data"]
    for measurement in data["measurements"]:
        points.append([])
        for point in measurement["lidar_data"]:
            if point["type"] != "point":
                continue
            points[-1].append(k @ point["coordinates"] + pos)
        d_rot = measurement["odometry"]["euler_angles"]
        if d_rot != [0, 0, 0]:
            rot += d_rot
            k = rot_m(rot)
        pos += k @ measurement["odometry"]["position"]

pcd = open3d.geometry.PointCloud()
pcd.points = open3d.utility.Vector3dVector(points[0])

vis = open3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)

run = True
while run:
    for p in points:
        vis.update_geometry(pcd)
        run = vis.poll_events()
        if not run:
            break
        vis.update_renderer()
        pcd.points = open3d.utility.Vector3dVector(p)
        time.sleep(0.03)
