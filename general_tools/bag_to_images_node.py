#!/usr/bin/env python

## subscribe to image topic(s) and write image to disk
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
cv2.ocl.setUseOpenCL(False)
from cv_bridge import CvBridge, CvBridgeError

import os

# OUT_BASE = '/Bulk_Data/ros_bags/tum-vins-dataset/tumvi/calibrated/512_16/calibration_bags/dataset-calib-cam1_512_16/'
OUT_BASE = '/Bulk_Data/ros_bags/mynteye/calib/calib2/'
print 'mkdir ', OUT_BASE
os.makedirs( OUT_BASE )

def callback(data):
    print( 'Received %s Image size=%d,%d' %(str(data.header.stamp), data.height, data.width ) )
    cv_image = CvBridge().imgmsg_to_cv2( data, 'bgr8' )
    cv2.imwrite( OUT_BASE+'/cam0_%s.png' %(str(data.header.stamp) ), cv_image )
    # cv2.imshow( 'win', cv_image )
    # cv2.waitKey(10)

def callback1(data):
    print( 'Received1 %s Image size=%d,%d' %(str(data.header.stamp), data.height, data.width ) )
    cv_image = CvBridge().imgmsg_to_cv2( data, 'bgr8' )
    cv2.imwrite( OUT_BASE+'/cam1_%s.png' %(str(data.header.stamp) ), cv_image )
    # cv2.imshow( 'win_1', cv_image )
    # cv2.waitKey(10)
    #

if __name__ == '__main__':
    rospy.init_node('bag_to_images_node', anonymous=True)
    # sub1 = rospy.Subscriber( "/cam0/image_raw", Image, callback)
    # sub2 = rospy.Subscriber( "/cam1/image_raw", Image, callback1)

    sub1 = rospy.Subscriber( "/mynteye/left/image_raw", Image, callback)
    sub2 = rospy.Subscriber( "/mynteye/right/image_raw", Image, callback1)

    print 'rospy.spin()'
    rospy.spin()
