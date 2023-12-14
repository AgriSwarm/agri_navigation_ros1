#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray

def callback(data):
    rospy.loginfo("Received MarkerArray with {} markers".format(len(data.markers)))

def listener():
    rospy.init_node('visualization_bridge_node', anonymous=True)
    rospy.Subscriber('/apple', MarkerArray, callback)
    rospy.Subscriber('/sensor_zone', MarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()