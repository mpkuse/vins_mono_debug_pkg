#!/usr/bin/python
#########
# This will subscribe to a topic which contains ground-truth.
# and publish the ground truths a linestrips.
#########

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry

## Ros Init
rospy.init_node('publish_gt', anonymous=True)


## Publisher
pub = rospy.Publisher('chatter_marker', Marker, queue_size=10)


## Subscriber
L = []
L_t = []

def publish_L():
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = 0.1
    #marker.scale.y = i
    #marker.scale.z = i
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 1.
    marker.pose.position.y = 0.
    marker.pose.position.z = 0.

    for i in range( len(L) ):
        pt = Point()
        pt.x = L[i][0]
        pt.y = L[i][1]
        pt.z = L[i][2]
        marker.points.append( pt )

    pub.publish(marker)


def callback(data):
    print 'callback', data.header.stamp.secs%1000, round(data.header.stamp.nsecs/1000.), "::>",
    print round(data.point.x,2), round(data.point.y,2), round(data.point.z,2)
    L.append( (data.point.x, data.point.y, data.point.z) )
    L_t.append( (data.header.stamp.secs, data.header.stamp.nsecs) )

    publish_L()


def callback_vins_camerapose(data):
    position = data.pose.pose.position
    print '\t\tcamerapose', data.header.stamp.secs%1000, round(data.header.stamp.nsecs/1000.), "::>",
    print round(position.x,2), round(position.y,2), round(position.z,2)


rospy.Subscriber("/leica/position", PointStamped, callback)
rospy.Subscriber("/vins_estimator/camera_pose", Odometry, callback_vins_camerapose)


rospy.spin()
