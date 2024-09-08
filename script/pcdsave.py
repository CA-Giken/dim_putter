#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import open3d as o3d
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from rovi_utils import tflib
import subprocess

Config={
  "ply":"capture.ply",
}


def cb_ps(msg):
  pvecs=np.reshape(msg.data,(-1,3))
  pcd=o3d.geometry.PointCloud()
  pcd.points=o3d.utility.Vector3dVector(pvecs)
  o3d.io.write_point_cloud(thispath+'/'+Config["ply"],pcd,True,False)
  print("pcd save",len(pvecs))
  return

########################################################
rospy.init_node("pcdsave",anonymous=True)
thispath=subprocess.getoutput("rospack find saisun3d")
###Load params
try:
  Config.update(rospy.get_param("/config/pcdsave"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
