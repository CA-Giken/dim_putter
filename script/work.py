#!/usr/bin/env python3

# Python includes
import numpy as np

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Vector3,Transform
from tf import transformations
from rovi_utils import tflib
from rviz_tools_py import rviz_tools

# Initialize the ROS Node
rospy.init_node('marker_work', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print("Shutting down node")
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('world', 'work_marker')
T1 = transformations.euler_matrix(0, 0, 0, axes='sxyz')
sc1 = Vector3(1,1,1)

while not rospy.is_shutdown():
  mesh_file1 = "package://saisun3d/mesh/TestPiece.stl"
  markers.publishMesh(T1,mesh_file1,'white', sc1, 0.5)
  rospy.Rate(5).sleep() #5Hz
