#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool
import numpy as np
import cv2



pub_restart = rospy.Publisher('/feature_tracker/restart', Bool, queue_size=10)
pub_rcvd = rospy.Publisher('/feature_tracker/rcvd_flag', Bool, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

im = np.zeros( (100,500), dtype='uint8' )
cv2.putText(im,'r: publish True  to /feature_tracker/restart',   (10,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
cv2.putText(im,'t: publish True  to /feature_tracker/rcvd_flag', (10,45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
cv2.putText(im,'f: publish False to /feature_tracker/rcvd_flag', (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)


while not rospy.is_shutdown():
    # hello_str = "hello world %s" % rospy.get_time()
    # rospy.loginfo(hello_str)
    print '---'
    print 'r: publish True  to /feature_tracker/restart'
    print 't: publish True  to /feature_tracker/rcvd_flag'
    print 'f: publish False to /feature_tracker/rcvd_flag'

    cv2.imshow( 'win', im )
    key = cv2.waitKey(10)
    if key == ord( 'r' ):
        print 'r pressed'
        pub_restart.publish(True)

    if key == ord( 't' ):
        print 't pressed'
        pub_rcvd.publish(True)
    if key == ord( 'f' ):
        print 'f pressed'
        pub_rcvd.publish(False)

    rate.sleep()
