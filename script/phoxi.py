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

import open3d as o3d
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2
from saisun3d import open3d_conversions

def cb_ps(msg):
  rawpc=open3d_conversions.from_msg(msg)
  print("phoxi pointcloud recieved",len(rawpc.points))
  rawpc.scale(1000,np.zeros(3))
  dwnpc=rawpc.voxel_down_sample(1.0)
  pc2=open3d_conversions.to_msg(dwnpc,frame_id="camera")
  pub_pc2.publish(pc2)
  return

# Initialize the ROS Node
rospy.init_node('phoxi_wrapper', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print("Shutting down node")
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('camera', 'camera_marker')
#T1 = transformations.translation_matrix((0,0,0))
T1 = transformations.euler_matrix(np.pi, np.pi/2, 0, axes='sxyz')
sc1 = Vector3(1,1,1)
tr2=Transform()
tr2.translation.z=450
tr2.rotation.w=1.
T2=tflib.toRT(tr2)
sc2 = Vector3(450,400,150)

###Topics
rospy.Subscriber("/phoxi_camera/pointcloud",PointCloud2,cb_ps)
pub_pc2=rospy.Publisher("/sensors/capt_pc2",PointCloud2,queue_size=1)

print("phoxi wrapper started")

while not rospy.is_shutdown():
  mesh_file1 = "package://saisun3d/mesh/PhoXi_S.f.STL"
  markers.publishMesh(T1,mesh_file1,'white', sc1, 0.5)
  markers.publishCube(T2,(0.1,0.1,0.1,0.05), sc2, 0.5)
  rospy.Rate(5).sleep() #5Hz
