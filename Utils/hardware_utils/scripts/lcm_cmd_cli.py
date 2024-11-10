#!/usr/bin/env python

import rospy
import sys
import termios
import tty
from geometry_msgs.msg import PoseStamped
from swarm_msgs.msg import CommandTOL, PositionCommand, IndexedOdometry
import numpy as np

class MavrosBridgeClient:
    def __init__(self):
        rospy.init_node('mavros_bridge_client', anonymous=True)
        
        # Publishers
        self.takeoff_command_pub = rospy.Publisher('/hardware_bridge/takeoff_mand', CommandTOL, queue_size=1)
        self.land_command_pub = rospy.Publisher('/hardware_bridge/land_mand', CommandTOL, queue_size=1)
        self.setpoint_position_pub = rospy.Publisher('/traj_server/setpoint_position', PositionCommand, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/d2vins/indexed_odometry', IndexedOdometry, self.odom_callback, queue_size=1)
        
        # Variables
        self.position_target = np.zeros(3)
        self.init_pose_pid = False
        self.odom_cur = None
        self.drone_id = 0  # ドローンID

    def odom_callback(self, msg):
        self.odom_cur = msg

    def publish_setpoint(self, dx=0.0, dy=0.0, dz=0.0):
        if self.odom_cur is None:
            print("No odometry data received yet.")
            return
        if not self.init_pose_pid:
            self.position_target = np.array([
                self.odom_cur.odom.pose.pose.position.x,
                self.odom_cur.odom.pose.pose.position.y,
                self.odom_cur.odom.pose.pose.position.z
            ])
            self.init_pose_pid = True

        # Update target position
        self.position_target += np.array([dx, dy, dz])

        # Publish setpoint
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.drone_id = self.drone_id
        cmd.pose.position.x = self.position_target[0]
        cmd.pose.position.y = self.position_target[1]
        cmd.pose.position.z = self.position_target[2]
        cmd.pose.orientation.w = 1.0
        self.setpoint_position_pub.publish(cmd)

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

    def publish_land_command(self):
        cmd = CommandTOL()
        cmd.drone_id = self.drone_id
        cmd.altitude = 0.0
        self.land_command_pub.publish(cmd)
        print("Land command published")

    def publish_takeoff_command(self):
        cmd = CommandTOL()
        cmd.drone_id = self.drone_id
        cmd.altitude = 1.0
        cmd.min_pitch = 0.0
        self.takeoff_command_pub.publish(cmd)
        print("Takeoff command published")

    def run(self):
        print("T: Takeoff, L: Land")
        print("Arrow keys: Move in x-y plane, K: Up, M: Down, Q: Quit")
        
        while True:
            char = self.getch().lower()
            
            if char == 't':
                self.publish_takeoff_command()
            elif char == 'l':
                self.publish_land_command()
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