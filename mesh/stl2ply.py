#!/usr/bin/env /usr/bin/python3

import open3d as o3d
import numpy as np
import copy
from scipy.spatial.transform import Rotation as rot

name="TestPiece"
mesh = o3d.io.read_triangle_mesh(name+".stl")
pcd=mesh.sample_points_uniformly(number_of_points=1000000)
pcdd=pcd.voxel_down_sample(voxel_size=1.0)
o3d.io.write_point_cloud(name+".ply",pcdd)
