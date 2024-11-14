#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool
import sys
import termios
import tty
from functools import partial
from mavros_msgs.srv import CommandTOL as CommandTOLSrv
from quadrotor_msgs.msg import GoalSet
from quadrotor_msgs.srv import UpdateMode, UpdateModeRequest
from hardware_utils.srv import RotateMotor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from swarm_msgs.msg import CommandTOL, PositionCommand, IndexedOdometry
import numpy as np

class MavrosBridgeClient:
    def __init__(self):
        rospy.init_node('mavros_bridge_client', anonymous=True)
        self.drone_id = rospy.get_param('~self_id', 0)
        self.use_lcm = rospy.get_param('~use_lcm', False)
        
        # Publishers
        # self.setpoint_position_pub = rospy.Publisher('/traj_server/setpoint_position', PositionCommand, queue_size=1)
        self.setpoint_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.target_marker_pub = rospy.Publisher('/cmd_gcs/setpoint_position_marker', Marker, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)

        if self.use_lcm:
            self.takeoff_command_pub = rospy.Publisher('/hardware_bridge/takeoff_mand', CommandTOL, queue_size=1)
            self.land_command_pub = rospy.Publisher('/hardware_bridge/land_mand', CommandTOL, queue_size=1)
            self.odom_sub = rospy.Subscriber('/d2vins/indexed_odometry', IndexedOdometry, self.lcm_odom_callback, queue_size=1)
        else:
            self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
            # self.odom_sub = rospy.Subscriber('/d2vins/odometry', Odometry, self.odom_callback)

        # Variables
        self.position_target = np.zeros(3)
        self.init_pose_pid = False
        self.activated = False
        self.odom_cur = None
        self.mode = 'setpoint_position'

    def lcm_odom_callback(self, msg):
        if msg.drone_id != self.drone_id:
            return
        self.odom_cur = msg

    def odom_callback(self, msg):
        self.odom_cur = msg

    def publish_setpoint(self, dx=0.0, dy=0.0, dz=0.0):
        if self.odom_cur is None:
            print("No odometry data received yet.")
            return
        # if not self.activated and not self.use_lcm:
        #     print("Drone not activated yet.")
        #     return
        if not self.init_pose_pid:
            if self.use_lcm:
                self.position_target = np.array([
                    self.odom_cur.odom.pose.pose.position.x,
                    self.odom_cur.odom.pose.pose.position.y,
                    self.odom_cur.odom.pose.pose.position.z
                ])
            else:
                self.position_target = np.array([
                    self.odom_cur.pose.pose.position.x,
                    self.odom_cur.pose.pose.position.y,
                    self.odom_cur.pose.pose.position.z
                ])
            self.init_pose_pid = True

        # Update target position
        self.position_target += np.array([dx, dy, dz])

        if self.mode == 'setpoint_position':
            # cmd = PositionCommand()
            # cmd.header.stamp = rospy.Time.now()
            # cmd.header.frame_id = "world"
            # cmd.drone_id = self.drone_id
            # cmd.pose.position.x = self.position_target[0]
            # cmd.pose.position.y = self.position_target[1]
            # cmd.pose.position.z = self.position_target[2]
            # cmd.pose.orientation.w = 1.0
            # self.setpoint_position_pub.publish(cmd)
            # print("Published setpoint: x=%.2f, y=%.2f, z=%.2f" % (
            #     self.position_target[0], 
            #     self.position_target[1], 
            #     self.position_target[2]
            # ))
            cmd = PoseStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "world"
            cmd.pose.position.x = self.position_target[0]
            cmd.pose.position.y = self.position_target[1]
            cmd.pose.position.z = self.position_target[2]
            cmd.pose.orientation.w = 1.0
            self.setpoint_position_pub.publish(cmd)
            print("Published setpoint: x=%.2f, y=%.2f, z=%.2f" % (
                self.position_target[0],
                self.position_target[1],
                self.position_target[2]
            ))
        else:  # goal mode
            goal_msg = GoalSet()
            goal_msg.drone_id = self.drone_id
            goal_msg.goal = [
                self.position_target[0],
                self.position_target[1],
                self.position_target[2]
            ]
            self.goal_pub.publish(goal_msg)
            print("Published goal: x=%.2f, y=%.2f, z=%.2f" % (
                self.position_target[0], 
                self.position_target[1], 
                self.position_target[2]
            ))

        # Publish visualization marker
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.position_target[0]
        marker.pose.position.y = self.position_target[1]
        marker.pose.position.z = self.position_target[2]
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.target_marker_pub.publish(marker)

    def handle_takeoff(self):
        if self.use_lcm:
            cmd = CommandTOL()
            cmd.drone_id = self.drone_id
            cmd.altitude = 1.0
            cmd.min_pitch = 0.0
            self.takeoff_command_pub.publish(cmd)
            self.activated = True
            print("Takeoff command published")
        else:
            result = self.call_takeoff_service()
            self.print_status("takeoff", result)

    def handle_land(self):
        if self.use_lcm:
            cmd = CommandTOL()
            cmd.drone_id = self.drone_id
            cmd.altitude = 0.0
            self.land_command_pub.publish(cmd)
            self.activated = False
            print("Land command published")
        else:
            result = self.call_land_service()
            self.print_status("land", result)

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
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOLSrv)
            response = takeoff_cl(altitude=1.0, latitude=0, longitude=0, min_pitch=0, yaw=0)
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
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOLSrv)
            response = takeoff_cl(altitude=1.0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

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

    def call_hover_service(self):
        service_name = '/update_mode'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            hover_service = rospy.ServiceProxy(service_name, UpdateMode)
            response = hover_service(UpdateModeRequest.HOVER)
            return response.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False
        
    def print_status(self, action, success):
        if success:
            print(f"Successfully {action} the MavrosBridge")
        else:
            print(f"Failed to {action} the MavrosBridge")

    @staticmethod
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch += sys.stdin.read(2)
                if ch == '\x1b[5' or ch == '\x1b[6':
                    ch += sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        print("Drone ID: %d" % self.drone_id)
        if self.use_lcm:
            print("Using LCM mode")
        else:
            print("Using non-LCM mode")
            print("A: Activate, D: Deactivate")
        print("T: Takeoff, L: Land")
        print("Arrow keys: Move in x-y plane")
        print("Page Up/Down: Move up/down")
        print("G: Switch to goal mode")
        print("P: Switch to setpoint position mode")
        print("Q: Quit")
        
        while True:
            char = self.getch()
            
            if not self.use_lcm:
                if char == 'a':
                    result = self.call_activate_service(True)
                    self.print_status("activated", result)
                elif char == 'd':
                    result = self.call_activate_service(False)
                    self.print_status("deactivated", result)
                elif char == 'h':
                    result = self.call_hover_service()
                    self.print_status("hover", result)
                elif char == 's':
                    result = self.call_shot_service()
                    self.print_status("shot", result)

            if char == 't':
                self.handle_takeoff()
            elif char == 'l':
                self.handle_land()
            elif char == 'g':
                self.mode = 'goal'
                print("Switched to goal mode")
            elif char == 'p':
                self.mode = 'setpoint_position'
                print("Switched to setpoint position mode")
            elif char == '\x1b[5~':  # Page Up
                self.publish_setpoint(dz=0.1)
            elif char == '\x1b[6~':  # Page Down
                self.publish_setpoint(dz=-0.1)
            elif char.startswith('\x1b['):
                if char == '\x1b[A':  # Up arrow
                    self.publish_setpoint(dx=0.1)
                elif char == '\x1b[B':  # Down arrow
                    self.publish_setpoint(dx=-0.1)
                elif char == '\x1b[C':  # Right arrow
                    self.publish_setpoint(dy=-0.1)
                elif char == '\x1b[D':  # Left arrow
                    self.publish_setpoint(dy=0.1)
            elif char == 'q':
                print("Quitting...")
                break

if __name__ == '__main__':
    client = MavrosBridgeClient()
    client.run()