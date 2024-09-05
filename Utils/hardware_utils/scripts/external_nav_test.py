#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


def circle_trajectory():
    rospy.init_node('external_nav_node', anonymous=True)
    odom_pub = rospy.Publisher('/mavros_bridge/odom', Odometry, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz

    # pose = PoseStamped()
    # pose.header.frame_id = "world"
    odom = Odometry()
    odom.header.frame_id = "world"

    # 初期位置
    x0, y0, z0 = 0, 0, 0
    radius = 1.0  # 円の半径
    angular_speed = 0.5  # 角速度 (rad/s)
    vertical_speed = 0.1  # 上昇速度 (m/s)

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - start_time).to_sec()
        angle = 0.0

        if dt < 10.0:
            # 10秒間停止
            odom.pose.pose.position.x = x0
            odom.pose.pose.position.y = y0
            odom.pose.pose.position.z = z0
        else:
            # 円を描きながら上昇
            angle = angular_speed * (dt - 10.0)
            odom.pose.pose.position.x = x0 + radius * math.cos(angle)
            odom.pose.pose.position.y = y0 + radius * math.sin(angle)
            odom.pose.pose.position.z = z0 + vertical_speed * (dt - 10.0)

        # ヨー角を円の接線方向に合わせる
        yaw = angle + math.pi/2
        q = quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.header.stamp = current_time

        odom_pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle_trajectory()
    except rospy.ROSInterruptException:
        pass