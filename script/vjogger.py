#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, String
from rovi_utils import tflib
from scipy.spatial.transform import Rotation as R
import time
import sys

Config={
  'tcp0_frame_id':'tool0_controller',
  'step_rz':15
}

degs=[30,30,-30,-30,-30,-30,30,30]

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def rot(euler,order='xyz'):
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=conf_tf[tcp0_id]['parent_frame_id']
  tf.child_frame_id=tcp0_id
  bTp=getRT(tf.header.frame_id,tf.child_frame_id)   # to tcp
  pTq=np.eye(4)
  pTq[:3,:3]=R.from_euler(order,euler,degrees=True).as_matrix()
  tf.transform=tflib.fromRT(bTp.dot(pTq))
  pub_tf.publish(tf);

def cb_jogrz(msg):
  step=Config["step_rz"]
  if not msg.data: step=-step
  rot([0,0,step])

rospy.init_node('vjogger',anonymous=True)
try:
  Config.update(rospy.get_param('/config/vjogger'))
except Exception as e:
  print("get_param exception:",e.args)

pub_tf=rospy.Publisher('/update/config_tf',TransformStamped,queue_size=1)
rospy.Subscriber("/driver/jog_rz",Bool,cb_jogrz)

tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False

###Start Event Loop##############
try:
  conf_tf=rospy.get_param('/config_tf')
except Exception as e:
  conf_tf={}
tcp0_id=Config['tcp0_frame_id']

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")

