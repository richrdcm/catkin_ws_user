import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub_gray = rospy.Publisher("/image_processing/gray_img",Image, queue_size=1)
    self.image_pub_bi = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # try:
        #   self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
        # except CvBridgeError as e:
        #   print(e)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    ret,bw_img=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

        # try:
        #   self.image_pub_bi.publish(self.bridge.cv2_to_imgmsg(bw_img, "mono8"))
        # except CvBridgeError as e:
        #   print(e)

    ##Getting the points from image##
    image_points = np.zeros((6,2));
    #image_points=[]

    for i in range (0,150):
        for j in range(0,320):
                  if (bw_img[i,j] >= 200):
                      image_points[0,0]=j
                      image_points[0,1]=i
                    #   coords = tuple([j,i])
                    #   image_points.append(coords)
                  break

    for i in range (150,300):
        for j in range (0,320):
               if (bw_img[i,j] >= 200):
                      image_points[1,0]=j
                      image_points[1,1]=i
                    #   coords = tuple([j,i])
                    #   image_points.append(coords)
               break

    for i in range(300,480):
       for j in range(0,320):
                 if (bw_img[i,j] >= 200):
                      image_points[2,0]=j
                      image_points[2,1]=i
                    #   coords = tuple([j,i])
                    #   image_points.append(coords)
                 break

    for i in range(0,150):
       for j in range(320,640):
                 if (bw_img[i,j] >= 200):
                      image_points[3,0]=j
                      image_points[3,1]=i
                    #   coords = tuple([j,i])
                    #   image_points.append(coords)
                 break

    for i in range(150,300):
        for j in range(320,640):
                  if (bw_img[i,j] >= 200):
                      image_points[4,0]=j
                      image_points[4,1]=i
                    #   coords = tuple([j,i])
                    #   image_points.append(coords)
                  break
    for i in range (300,480):
        for j in range(320,640):
               if (bw_img[i,j] >= 200):
                      image_points[5,0]=j
                      image_points[5,1]=i
                    #   coords = tuple([j,i])
                    #   image_points.append(coords)
               break

    #img_points = np.asanarray(image_points)
    #Define the world coordinates

    #world_points[0[0[0]]]
    world_points=np.array([[0,80,0],[0,40,0],[0,0,0],[28,80,0],[28,40,0],[28,0,0]],np.float32)
    # coords = tuple([0,80,0])
    # world_points.append(coords)
    # coords = tuple([0,40,0])
    # world_points.append(coords)
    # coords = tuple([0,0,0])
    # world_points.append(coords)
    # coords = tuple([28,80,0])
    # world_points.append(coords)
    # coords = tuple([28,40,0])
    # world_points.append(coords)
    # coords = tuple([28,0,0])
    # world_points.append(coords)

    intrinsics = np.array([[614.1699, 0, 329.9491], [0, 614.9002, 237.2788], [ 0, 0, 1]], np.float32)
    #intrinsics = []
    #intrinsics = (Mat_<double>(3,3) << 614.1699, 0, 329.9491, 0, 614.9002, 237.2788, 0, 0, 1);

    distCoeffs = np.array([0.1115,-0.1089,0,0],np.float32)
    #distCoeffs(4,1,double);
    # distCoeffs[0] = 0.1115;
    # distCoeffs[1] = -0.1089;
    # distCoeffs[2] = 0;
    # distCoeffs[3] = 0;

    rvec = np.zeros((3,1))
    tvec = np.zeros((3,1))

    cv2.solvePnP(world_points, image_points, intrinsics, distCoeffs, rvec, tvec);

    rmat = np.zeros((3,3))
    cv2.Rodrigues(rmat, rvec)
    print rmat
    #Inverting the matrix [ R | t ]^-1 = [R' | -R'*t]#
    #inv_rmat = np.array([[0, 0, 0], [0, 0, 0], [ 0, 0, 0]], np.float32)
    inv_rmat = rmat.transpose




    #
    # inv_tvec = np.zeros((3,1))
    # inv_tvec[0] = inv_rmat[0,0]*tvec[0]+inv_rmat[0,1]*tvec[1]+inv_rmat[0,2]*tvec[2]
    #
    # #np.matmul(inv_rmat,tvec)
    #
    # sy = sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0]);
    #
    # singular = sy < 1e-6; # If
    #
    # if (~singular):
    #      x = atan2(rmat[2,1] , rmat[2,2]);
    #      y = atan2(-rmat[2,0], sy);
    #      z = atan2(rmat[1,0], rmat.at[0,0]);
    # else:
    #      x = atan2(-rmat[1,2], rmat[1,1]);
    #      y = atan2(-rmat[2,0], sy);
    #      z = 0;
    #
    # print image_points
    # print world_points
    # print rvec
    # print tvec
    # print rmat
    # print inv_tvec
    # print x,y,z

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
