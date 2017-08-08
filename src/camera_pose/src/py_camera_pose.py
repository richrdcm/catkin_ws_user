#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
import math
import tf


def main(args):
  #ic = image_converter()
  rospy.init_node('konrad_pose', anonymous=True)
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
          rospy.init_node('konrad_pose', anonymous=True)
          pub_pose = rospy.Publisher("konrad/pose",Pose, queue_size=1)
          Master = Pose()
          Master.position.x = 0.4
          Master.position.y = 0.8
          Master.position.z = 0
          q = tf.transformations.quaternion_from_euler(2.08248210007, 0.00907519397895,  0.0311507585032)
          Master.orientation.x = q[0]
          Master.orientation.y = q[1]
          Master.orientation.z = q[2]
          Master.orientation.w = q[3]
          pub_pose.publish(Master)

          print 'Pose' , Master
          print 'Pose' , Master
          br = tf.TransformBroadcaster()
          br.sendTransform((0.4,0.8,0),
                     q,
                     rospy.Time(0),
                     "konrad",
                     "world")

                     #rospy.spin()
          rate.sleep() #OJO


if __name__ == '__main__':
    main(sys.argv)
