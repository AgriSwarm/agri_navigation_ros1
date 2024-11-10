#!/usr/bin/env python

import rospy
import sys
import termios
import tty
from geometry_msgs.msg import PoseStamped
from swarm_msgs.msg import CommandTOL, PositionCommand, IndexedOdometry
from quadrotor_msgs.msg import GoalSet
import numpy as np

class MavrosBridgeClient:
    def __init__(self):
        rospy.init_node('mavros_bridge_client', anonymous=True)
        
        # Get drone_id from rosparam
        self.drone_id = rospy.get_param('~self_id', 0) 
        
        # Publishers
        self.takeoff_command_pub = rospy.Publisher('/hardware_bridge/takeoff_mand', CommandTOL, queue_size=1)
        self.land_command_pub = rospy.Publisher('/hardware_bridge/land_mand', CommandTOL, queue_size=1)
        self.setpoint_position_pub = rospy.Publisher('/traj_server/setpoint_position', PositionCommand, queue_size=1)
        self.goal_publisher = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/d2vins/indexed_odometry', IndexedOdometry, self.odom_callback, queue_size=1)
        
        # Variables
        self.position_target = np.zeros(3)
        self.init_pose_pid = False
        self.activated = False
        self.odom_cur = None
        self.mode = 'setpoint_position'

    def odom_callback(self, msg):
        if msg.drone_id != self.drone_id:
            return
        self.odom_cur = msg

    def publish_setpoint(self, dx=0.0, dy=0.0, dz=0.0):
        if self.odom_cur is None:
            print("No odometry data received yet.")
            return
        if not self.activated:
            print("Drone not activated yet.")
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

        if self.mode == 'setpoint_position':
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
            self.goal_publisher.publish(goal_msg)
            print("Published goal: x=%.2f, y=%.2f, z=%.2f" % (
                self.position_target[0], 
                self.position_target[1], 
                self.position_target[2]
            ))

    def publish_land_command(self):
        cmd = CommandTOL()
        cmd.drone_id = self.drone_id
        cmd.altitude = 0.0
        self.land_command_pub.publish(cmd)
        self.activated = False
        print("Land command published")

    def publish_takeoff_command(self):
        cmd = CommandTOL()
        cmd.drone_id = self.drone_id
        cmd.altitude = 1.0
        cmd.min_pitch = 0.0
        self.takeoff_command_pub.publish(cmd)
        self.activated = True
        print("Takeoff command published")

    # getch関数を修正して、より長いキーシーケンスを処理できるようにします
    @staticmethod
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch += sys.stdin.read(2)
                if ch == '\x1b[5' or ch == '\x1b[6':  # Page Up/Down の最初の部分
                    ch += sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        print("Drone ID: %d" % self.drone_id)
        print("T: Takeoff, L: Land")
        print("Arrow keys: Move in x-y plane")
        print("Page Up: Move up, Page Down: Move down")
        print("G: Switch to goal mode, P: Switch to setpoint_position mode")
        print("Q: Quit")
        
        while True:
            char = self.getch()
            
            if char == 'g':
                self.mode = 'goal'
                print("Switched to goal mode")
            elif char == 'p':
                self.mode = 'setpoint_position'
                print("Switched to setpoint_position mode")
            elif char == 't':
                self.publish_takeoff_command()
            elif char == 'l':
                self.publish_land_command()
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