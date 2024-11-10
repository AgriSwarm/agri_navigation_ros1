#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool
import sys
import termios
import tty
from functools import partial
from mavros_msgs.srv import CommandTOL
from quadrotor_msgs.msg import GoalSet
from quadrotor_msgs.srv import UpdateMode, UpdateModeRequest
from hardware_utils.srv import RotateMotor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np

class MavrosBridgeClient:
    def __init__(self):
        rospy.init_node('mavros_bridge_client', anonymous=True)
        
        # Publishers
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.marker_publisher = rospy.Publisher('setpoint_position_marker', Marker, queue_size=10)
        self.goal_publisher = rospy.Publisher('/goal_local_with_id', GoalSet, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        
        # Variables
        self.position_target = np.zeros(3)
        self.init_pose_pid = False
        self.odom_cur = None
        
        # Timer for continuous setpoint publishing (commented out)
        # self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def odom_callback(self, msg):
        self.odom_cur = msg

    def publish_setpoint(self, dx=0.0, dy=0.0, dz=0.0):
        if self.odom_cur is None:
            print("No odometry data received yet.")
            return
        if not self.init_pose_pid:
            self.position_target = np.array([
                self.odom_cur.pose.pose.position.x,
                self.odom_cur.pose.pose.position.y,
                self.odom_cur.pose.pose.position.z
            ])
            self.init_pose_pid = True

        # Update target position
        self.position_target += np.array([dx, dy, dz])

        # Publish setpoint
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.position_target[0]
        pose_msg.pose.position.y = self.position_target[1]
        pose_msg.pose.position.z = self.position_target[2]
        pose_msg.pose.orientation.w = 1.0
        self.setpoint_publisher.publish(pose_msg)

        # Publish visualization marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)

    def timer_callback(self, event):
        if self.init_pose_pid:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = self.position_target[0]
            pose_msg.pose.position.y = self.position_target[1]
            pose_msg.pose.position.z = self.position_target[2]
            pose_msg.pose.orientation.w = 1.0
            self.setpoint_publisher.publish(pose_msg)

    @staticmethod
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def call_activate_service(self, activate):
        service_name = '/mavros_bridge/activate'
        
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            activate_service = rospy.ServiceProxy(service_name, SetBool)
            response = activate_service(activate)
            return response.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def call_land_service(self):
        service_name = '/mavros/cmd/land'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            response = takeoff_cl(altitude=1, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def call_takeoff_service(self):
        service_name = '/mavros/cmd/takeoff'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_cl(altitude=1.0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def print_status(self, action, success):
        if success:
            print(f"Successfully {action} the MavrosBridge")
        else:
            print(f"Failed to {action} the MavrosBridge")

    def call_shot_service(self):
        service_name = 'rotate_motor'
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        try:
            shot_service = rospy.ServiceProxy(service_name, RotateMotor)
            response = shot_service(duration=1.0)
            return response.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def publish_goal(self, x, y, z):
        goal_msg = GoalSet()
        goal_msg.drone_id = 0
        goal_msg.goal = [x, y, z]
        self.goal_publisher.publish(goal_msg)
        print(f"Published goal: x={x}, y={y}, z={z}")

    def call_hover_service(self):
        service_name = '/update_mode'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            hover_service = rospy.ServiceProxy(service_name, UpdateMode)
            response = hover_service(UpdateModeRequest.HOVERING)
            return response.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def run(self):
        print("A: Activate, D: Deactivate, T: Takeoff, L: Land, H: Hover")
        print("Arrow keys: Move in x-y plane, Q: Quit")
        
        while True:
            char = self.getch().lower()
            print(f"Pressed: {char}")
            print(f"char == '1': {char == '1'}")
            
            if char == 'a':
                result = self.call_activate_service(True)
                self.print_status("activated", result)
            elif char == 'd':
                result = self.call_activate_service(False)
                self.print_status("deactivated", result)
            elif char == 't':
                result = self.call_takeoff_service()
                self.print_status("takeoff", result)
            elif char == 'l':
                result = self.call_land_service()
                self.print_status("land", result)
            elif char == 's':
                result = self.call_shot_service()
                self.print_status("shot", result)
            elif char == 'h':
                result = self.call_hover_service()
                self.print_status("hover", result)
            elif char == 'k':
                self.publish_setpoint(dz=0.1)
            elif char == 'm':
                self.publish_setpoint(dz=-0.1)
            elif char == '\x1b':
                next1, next2 = self.getch(), self.getch()
                if next1 == '[':
                    if next2 == 'A':  # Up arrow
                        self.publish_setpoint(dx=0.1)
                    elif next2 == 'B':  # Down arrow
                        self.publish_setpoint(dx=-0.1)
                    elif next2 == 'C':  # Right arrow
                        self.publish_setpoint(dy=-0.1)
                    elif next2 == 'D':  # Left arrow
                        self.publish_setpoint(dy=0.1)
            elif char == 'q':
                print("Quitting...")
                break

if __name__ == '__main__':
    client = MavrosBridgeClient()
    client.run()