#!/usr/bin/env python3

import numpy as np
import os
import sys
import subprocess
import copy
import open3d as o3d
import roslib
import rospy
import tf
import tf2_ros
from scipy.spatial.transform import Rotation as rot
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Config={
  "model":"mesh/TestPiece.ply",
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def cb_redraw(msg):
  global pModel
  pModel=o3d.io.read_point_cloud(thispath+'/'+Config['model'])
  print("vstacker::points",len(pModel.points))
  pub_wp.publish(np2F(np.array(pModel.points)))

def cb_loadPcd(ev):
  global pModel
  pModel=o3d.io.read_point_cloud(thispath+'/'+Config['model'])

def cb_clear(msg):
  pass

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

########################################################
rospy.init_node("vstacker",anonymous=True)
thispath=subprocess.getoutput("rospack find dim_putter")
###Load params
try:
  Config.update(rospy.get_param("/config/vscener"))
except Exception as e:
  print("get_param exception:",e.args)
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:
#  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_wp=rospy.Publisher("/virtual/scene_floats",numpy_msg(Floats),queue_size=1)
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
###Global
mError=Int32()
rospy.Timer(rospy.Duration(10),cb_loadPcd,oneshot=True)

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
