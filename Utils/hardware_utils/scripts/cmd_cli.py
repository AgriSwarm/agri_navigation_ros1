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

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def call_activate_service(activate):
    service_name = '/mavros_bridge/activate'
    
    try:
        rospy.wait_for_service(service_name, timeout=5.0)  # 5秒のタイムアウトを設定
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
    
def call_land_service():
    service_name = '/mavros/cmd/land'

    try:
        rospy.wait_for_service(service_name, timeout=5.0)  # 5秒のタイムアウトを設定
    except rospy.ROSException:
        print(f"Service {service_name} is not available. Timeout occurred.")
        return False
    
    try:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = takeoff_cl(altitude=1, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False
    
def call_takeoff_service():
    service_name = '/mavros/cmd/takeoff'

    try:
        rospy.wait_for_service(service_name, timeout=5.0)  # 5秒のタイムアウトを設定
    except rospy.ROSException:
        print(f"Service {service_name} is not available. Timeout occurred.")
        return False
    
    try:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        response = takeoff_cl(altitude=1, latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def print_status(action, success):
    if success:
        print(f"Successfully {action} the MavrosBridge")
    else:
        print(f"Failed to {action} the MavrosBridge")

def publish_goal(publisher, x, y, z):
    goal_msg = GoalSet()
    goal_msg.drone_id = 0
    goal_msg.goal = [x, y, z]
    publisher.publish(goal_msg)
    print(f"Published goal: x={x}, y={y}, z={z}")

def call_hover_service():
    service_name = '/traj_server/update_mode'

    try:
        rospy.wait_for_service(service_name, timeout=5.0)  # 5秒のタイムアウトを設定
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

if __name__ == '__main__':
    rospy.init_node('mavros_bridge_client', anonymous=True)
    goal_publisher = rospy.Publisher('/goal_local_with_id', GoalSet, queue_size=10)
    
    print("A: Activate, D: Deactivate, T: Takeoff, L: Land, H: Hover")
    print("Arrow keys: Move in x-y plane, Q: Quit")
    
    while True:
        char = getch().lower()
        
        if char == 'a':
            result = call_activate_service(True)
            print_status("activated", result)
        elif char == 'd':
            result = call_activate_service(False)
            print_status("deactivated", result)
        elif char == 't':
            result = call_takeoff_service()
            print_status("takeoff", result)
        elif char == 'l':
            result = call_land_service()
            print_status("land", result)
        elif char == 'h':
            result = call_hover_service()
            print_status("hover", result)
        elif char == '\x1b':
            next1, next2 = getch(), getch()
            if next1 == '[':
                if next2 == 'A':  # Up arrow
                    publish_goal(goal_publisher, 0.5, 0, 0)
                elif next2 == 'B':  # Down arrow
                    publish_goal(goal_publisher, -0.5, 0, 0)
                elif next2 == 'C':  # Right arrow
                    publish_goal(goal_publisher, 0, 0.5, 0)
                elif next2 == 'D':  # Left arrow
                    publish_goal(goal_publisher, 0, -0.5, 0)
        elif char == 'q':
            print("Quitting...")
            break
        else:
            print("Invalid input. Press 'A' to activate, 'D' to deactivate, 'T' to takeoff, 'L' to land, arrow keys to move, or 'Q' to quit.")
