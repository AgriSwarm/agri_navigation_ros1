#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool
import sys
import termios
import tty
from functools import partial
from mavros_msgs.srv import CommandTOL

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

if __name__ == '__main__':
    rospy.init_node('mavros_bridge_client', anonymous=True)
    print("Press 'A' to activate, 'D' to deactivate, or 'Q' to quit.")
    
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
        elif char == 'q':
            print("Quitting...")
            break
        else:
            print("Invalid input. Press 'A' to activate, 'D' to deactivate, 'T' to takeoff, 'L' to land or 'Q' to quit.")