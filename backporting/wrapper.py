#!/usr/bin/env python2

# Python includes
import numpy as np

# ROS includes
import roslib
import rospy
from std_msgs.msg import Bool
from phoxi_camera.srv import TriggerImage,TriggerImageRequest,TriggerImageResponse
from phoxi_camera.srv import GetFrame,GetFrameRequest

def cb_capture(msg):
  print "phoxi wrapper::capture"
  rospy.wait_for_service('/phoxi_camera/trigger_image')
  try:
    trigger_image = rospy.ServiceProxy('/phoxi_camera/trigger_image',TriggerImage)
    get_frame = rospy.ServiceProxy('/phoxi_camera/get_frame',GetFrame)
#    res = trigger_image(TriggerImageRequest())
    res = trigger_image()
    req = GetFrameRequest()
    req.in_ = res.id
    get_frame(req)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

# Initialize the ROS Node
rospy.init_node('phoxi_wrapper', anonymous=False, log_level=rospy.INFO, disable_signals=False)

###Topics
rospy.Subscriber("/sensors/X1",Bool,cb_capture)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
