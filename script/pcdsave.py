#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import open3d as o3d
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2
from saisun3d import open3d_conversions
from rovi_utils import tflib
import subprocess

Config={
  "ply":"capture.ply",
}


def cb_ps(msg):
  pcd=open3d_conversions.from_msg(msg)
  o3d.io.write_point_cloud(thispath+'/'+Config["ply"],pcd,True,False)
  print("pcd save",len(pcd.points))
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
rospy.Subscriber("/sensors/capt_pc2",PointCloud2,cb_ps)
###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
