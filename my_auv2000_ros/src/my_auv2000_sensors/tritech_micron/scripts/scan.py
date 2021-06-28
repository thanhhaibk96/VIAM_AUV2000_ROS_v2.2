#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Tritech Micron sonar scanner.

This publishes one PointCloud message per scan slice. In order to visualize in
rviz, play with the 'Decay Time' parameter. This node also provides parameters
that can be dynamically reconfigured.
"""
## Import noi dung nay de publish 1 buc anh:
##########################################
import os#################################
import cv2################################
from std_msgs.msg import Header###########
from sensor_msgs.msg import Image#########
##########################################

import time
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import matplotlib.pyplot as plt
from multiprocessing import Process
from matplotlib.animation import FuncAnimation

from sensor_msgs.msg import PointCloud
from tritech_micron import TritechMicron
from geometry_msgs.msg import PoseStamped
from tritech_micron.cfg import ScanConfig
from dynamic_reconfigure.server import Server
from tritech_micron.msg import TritechMicronConfig

__author__ = "Anass Al-Wohoush"


pos = np.zeros((399*200,5))

def reconfigure(config, level):
    """Reconfigures sonar dynamically.

    Args:
        config: New configuration.
        level: Level bitmask.

    Returns:
        Configuration.
    """
    rospy.loginfo("Reconfiguring sonar")
    rospy.logdebug("Configuration requested: %r, %r", config, level)

    # Remove additional keys.
    if "groups" in config:
        config.pop("groups")

    # Set parameters.
    sonar.set(**config)
    return config



def scale_to_255(a, min, max, dtype=np.uint8):
    """ Scales an array of values from specified min, max range to 0-255
        Optionally specify the data type of the output (default is uint8)
    """
    return (((a - min) / float(max - min)) * 255).astype(dtype)


def publish(sonar, slice):
    """Publishes PointCloud, PoseStamped and TritechMicronConfig of current
    scan slice on callback.

    Args:
        sonar: Sonar instance.
        slice: Current scan slice.
    """

    # Publish heading as PoseStamped.
    posestamped = slice.to_posestamped(frame)
    heading_pub.publish(posestamped)
    my_heading = slice.heading
    #rospy.loginfo(slice.heading)

    # Publish data as PointCloud.
    cloud = slice.to_pointcloud(frame)
    scan_pub.publish(cloud)
    #rospy.loginfo(cloud)

    ## Init pos matrix, i want to run this cmd only one time.
    j = int(my_heading/6.283184307*200)
    rospy.loginfo(j)
    for i in range(0, len(cloud.points)-1):
        pos[i+399*j,0] = cloud.points[i].x
        pos[i+399*j,1] = cloud.points[i].y
        pos[i+399*j,2] = cloud.points[i].z
        pos[i+399*j,3] = my_heading
        pos[i+399*j,4] = cloud.channels[0].values[i]
    rospy.loginfo(my_heading)

    if j == 0:
        np.savetxt("/home/lianquan/Desktop/file.csv", pos, delimiter=",")

        ####################################################################################################
        side_range = ( -10.0 , 10.0)     # left-most to right-most
        fwd_range  = ( -10.0 , 10.0)       # back-most to forward-most
        # EXTRACT THE POINTS FOR EACH AXIS
        x_points = pos[:, 0]
        y_points = pos[:, 1]
        z_points = pos[:, 2]
        # FILTER - To return only indices of points within desired cube
        # Three filters for: Front-to-back, side-to-side, and height ranges
        # Note left side is positive y axis in LIDAR coordinates
        f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
        s_filt = np.logical_and((y_points > -side_range[1]), (y_points < -side_range[0]))
        filter = np.logical_and(f_filt, s_filt)
        indices = np.argwhere(filter).flatten()
        # KEEPERS
        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]

        res = 0.05

        # CONVERT TO PIXEL POSITION VALUES - Based on resolution
        x_img = (-y_points/res).astype(np.int32)  # x axis is
        y_img = (-x_points/res).astype(np.int32)  # y axis is

        # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
        # floor and ceil used to prevent anything being rounded to below 0 after shift
        x_img -= int(np.floor(side_range[0] / res))
        y_img += int(np.ceil(fwd_range[1] / res))
        pixel_values  = pos[:, 4]
        # INITIALIZE EMPTY ARRAY - of the dimensions we want
        x_max = (1+int((side_range[1] - side_range[0])/res))
        y_max = (1+int((fwd_range[1] - fwd_range[0])/res))
        im9 = np.zeros([y_max, x_max], dtype=np.uint8)
        # FILL PIXEL VALUES IN IMAGE ARRAY

        rospy.loginfo(im9)
        im9[y_img, x_img] = pixel_values
        np.savetxt("/home/lianquan/Desktop/im9.csv", im9, delimiter=",")
        import matplotlib.pyplot as plt
        plt.imshow(im9, cmap="nipy_spectral", vmin=0, vmax=255)
        plt.show()
        plt.savefig("/home/lianquan/Desktop/sonar_color.jpeg")

        #from PIL import Image
        #im2 = Image.fromarray(im9)
        #im2.show()
        #im2.save("/home/lianquan/Desktop/sonar.png")
        #np.savetxt("/home/lianquan/Desktop/im2.csv", im2, delimiter=",")

####################################################################################

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    (cols,rows) = cv.GetSize(cv_image)
    if cols > 60 and rows > 60 :
      cv.Circle(cv_image, (50,50), 10, 255)

    cv.ShowImage("Image window", cv_image)
    cv.WaitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e



if __name__ == "__main__":
    # Initialize node and publishers.
    rospy.init_node("tritech_micron")
    scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)
    conf_pub = rospy.Publisher("~config", TritechMicronConfig, queue_size=800)

    # Get frame name and port.
    frame = rospy.get_param("~frame")
    port = rospy.get_param("~port")

    with TritechMicron(port=port) as sonar:
        try:
            # Initialize dynamic reconfigure server and scan.
            Server(ScanConfig, reconfigure)

            # Scan.
            sonar.scan(callback=publish)
        except KeyboardInterrupt:
            sonar.preempt()
