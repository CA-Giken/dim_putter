#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import open3d as o3d
import json
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from rovi_utils import tflib
from sensor_msgs.msg import PointCloud2
from saisun3d import open3d_conversions

Config={
}
Param={
}

def cb_ps(msg):
  global Scene
  Scene=open3d_conversions.from_msg(msg)
  print("saisun sub scene",len(Scene.points))
  rospy.loginfo("saisun sub",len(Scene.points))

  result={"length":100}
  pub_result.publish(json.dumps(result))
  return

########################################################
rospy.init_node("saisun",anonymous=True)
###Load params
try:
  Config.update(rospy.get_param("/config/saisun"))
except Exception as e:
  print("get_param exception:",e.args)
###Subscriber
rospy.Subscriber("~scene",PointCloud2,cb_ps)
###Publishers(Planes as PointCloud2)
pub_p0=rospy.Publisher("/saisun/p0",PointCloud2,queue_size=1)
pub_p1=rospy.Publisher("/saisun/p1",PointCloud2,queue_size=1)
pub_p2=rospy.Publisher("/saisun/p2",PointCloud2,queue_size=1)
pub_p3=rospy.Publisher("/saisun/p3",PointCloud2,queue_size=1)
###Publishers(Center axis as PoseArray)
pub_ax0=rospy.Publisher("/saisun/ax0",PoseArray,queue_size=1) #axis of p0
pub_ax1=rospy.Publisher("/saisun/ax1",PoseArray,queue_size=1) #axis of p1
pub_ax2=rospy.Publisher("/saisun/ax2",PoseArray,queue_size=1) #axis of p2
pub_ax3=rospy.Publisher("/saisun/ax3",PoseArray,queue_size=1) #axis of p3
###Publishers(Edges as PoseArray)
pub_eg0=rospy.Publisher("/saisun/eg0",PoseArray,queue_size=1) #edge of p0 
pub_eg1=rospy.Publisher("/saisun/eg1",PoseArray,queue_size=1) #edge of p1
pub_eg2=rospy.Publisher("/saisun/eg2",PoseArray,queue_size=1) #edge of p2
pub_eg3=rospy.Publisher("/saisun/eg3",PoseArray,queue_size=1) #edge of p3

###Publishers(Results as JSON)
pub_result=rospy.Publisher("/saisun/result",String,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
