#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import gps
import time
import signal
import sys
from geopy import distance
import math

class GPSPublisher:
    def __init__(self):
        rospy.init_node('gps_publisher', anonymous=True)
        self.pub_fix = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        self.pub_position = rospy.Publisher('gps/position', PointStamped, queue_size=10)
        self.pub_path = rospy.Publisher('gps/path', Path, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.session = None
        self.origin = None
        self.origin_altitude = None
        self.path = Path()
        self.path.header.frame_id = "map"

    def connect_gps(self):
        try:
            self.session = gps.gps(mode=gps.WATCH_ENABLE)
            rospy.loginfo("Connected to GPS")
        except Exception as e:
            rospy.logerr("Failed to connect to GPS: %s", str(e))
            sys.exit(1)

    def publish_gps_data(self):
        while not rospy.is_shutdown():
            try:
                report = self.session.next()
                if report['class'] == 'TPV':
                    if hasattr(report, 'lat') and hasattr(report, 'lon'):
                        msg = NavSatFix()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = "gps"
                        msg.latitude = report.lat
                        msg.longitude = report.lon
                        msg.altitude = getattr(report, 'alt', float('nan'))
                        
                        self.pub_fix.publish(msg)
                        rospy.loginfo("Published: Lat: %f, Lon: %f, Alt: %f", msg.latitude, msg.longitude, msg.altitude)

                        # Publish position in meters
                        if self.origin is None:
                            self.origin = (msg.latitude, msg.longitude)
                            self.origin_altitude = msg.altitude if not math.isnan(msg.altitude) else 0.0
                            rospy.loginfo("Origin set to: Lat: %f, Lon: %f, Alt: %f", self.origin[0], self.origin[1], self.origin_altitude)
                        
                        position = PointStamped()
                        position.header.stamp = rospy.Time.now()
                        position.header.frame_id = "map"
                        position.point.x = distance.distance(self.origin, (msg.latitude, self.origin[1])).meters
                        if msg.longitude < self.origin[1]:
                            position.point.x = -position.point.x
                        position.point.y = distance.distance(self.origin, (self.origin[0], msg.longitude)).meters
                        if msg.latitude < self.origin[0]:
                            position.point.y = -position.point.y
                        position.point.z = msg.altitude - self.origin_altitude if not math.isnan(msg.altitude) else 0.0

                        self.pub_position.publish(position)
                        rospy.loginfo("Published position: x: %f, y: %f, z: %f", position.point.x, position.point.y, position.point.z)

                        # Add point to path and publish
                        pose = PoseStamped()
                        pose.header = position.header
                        pose.pose.position = position.point
                        pose.pose.orientation.w = 1.0
                        self.path.poses.append(pose)
                        self.path.header.stamp = rospy.Time.now()
                        self.pub_path.publish(self.path)
                        rospy.loginfo("Published path with %d points", len(self.path.poses))

            except KeyError:
                pass
            except StopIteration:
                rospy.logwarn("No GPS data available")
            except Exception as e:
                rospy.logerr("Error reading GPS data: %s", str(e))

            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down GPS publisher")
        if self.session:
            self.session.close()
        rospy.signal_shutdown("GPS publisher shutting down")

def signal_handler(signum, frame):
    rospy.loginfo("Received shutdown signal")
    gps_pub.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    gps_pub = GPSPublisher()
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        gps_pub.connect_gps()
        gps_pub.publish_gps_data()
    except rospy.ROSInterruptException:
        pass
    finally:
        gps_pub.shutdown()