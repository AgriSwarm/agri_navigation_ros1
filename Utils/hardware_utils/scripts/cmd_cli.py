#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool
import sys
import termios
import tty
from functools import partial
from std_msgs.msg import Empty
from mavros_msgs.srv import CommandTOL as CommandTOLSrv
from quadrotor_msgs.msg import GoalSet
from quadrotor_msgs.srv import UpdateMode, UpdateModeRequest
from hardware_utils.srv import RotateMotor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from swarm_msgs.msg import CommandTOL, PositionCommand, IndexedOdometry
import numpy as np

import math
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from quadrotor_msgs.msg import TrackingPose, GoalSet
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix, quaternion_multiply

from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Polygon, PolygonStamped, Point32

class DemoManager:
    def __init__(self, init_odom, drone_id):
        # パラメータ設定
        self.dummy_flower_trigger_radius = 1.5
        self.route_track_trigger_radius = 0.5
        self.dummy_target_rel_position = [5.0, 0.0, 1.0]
        euler = [0.0, 1.0, 0.5]
        self.dummy_target_rel_orientation = R.from_euler('xyz', euler).as_quat()
        self.tracking_distance = 0.5
        self.drone_id = drone_id
        self.initialized = False

        self.goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
        self.escape_pub = rospy.Publisher('/escape_with_id', GoalSet, queue_size=10)
        self.tracking_pose_pub = rospy.Publisher("/traj_server/planning/track_pose", TrackingPose, queue_size=10)
        self.marker_pub = rospy.Publisher("/cmd_gcs/flower_detection_spheres", Marker, queue_size=10)
        self.dummy_target_pub = rospy.Publisher("/cmd_gcs/dummy_target_position", PoseStamped, queue_size=10)
        self.structure_polygon_pub = rospy.Publisher("/cmd_gcs/structure_polygon", PolygonStamped, queue_size=10)
        self.mand_start_pub = rospy.Publisher('/mandatory_start_to_planner', Empty, queue_size=10)

        self.transform_by_odom(init_odom)

        self.publish_markers()
    
    def fix_target(self, odom, flag=True):
        self.initialized = flag
        self.transform_by_odom(odom)

    def transform_by_odom(self, odom):
        if self.initialized:
            return
        self.init_pos = odom.pose.pose.position
        self.init_ori = odom.pose.pose.orientation

        # 初期姿勢のクォータニオンから回転行列を作成
        rot_matrix = quaternion_matrix([self.init_ori.x, self.init_ori.y, self.init_ori.z, self.init_ori.w])

        # 相対位置ベクトルを回転させる
        rel_pos_vec = [self.dummy_target_rel_position[0],
                      self.dummy_target_rel_position[1],
                      self.dummy_target_rel_position[2],
                      1.0]  # 同次座標として1を追加
        
        # 回転行列をかけて変換
        transformed_pos = rot_matrix.dot(rel_pos_vec)
        # transformed_pos = rel_pos_vec

        # 変換後の位置を初期位置に加算
        dummy_x = self.init_pos.x + transformed_pos[0]
        dummy_y = self.init_pos.y + transformed_pos[1]
        dummy_z = self.init_pos.z + transformed_pos[2]

        # dummy_targetのPose生成
        self.dummy_target_pose = Pose()
        self.dummy_target_pose.position = Point(dummy_x, dummy_y, dummy_z)
        # Orientationは相対姿勢をそのまま使用（実際にはinit_odomの姿勢と組み合わせが必要な場合もあるが、ここでは簡略化）
        self.dummy_target_pose.orientation = Quaternion(
            self.dummy_target_rel_orientation[0],
            self.dummy_target_rel_orientation[1],
            self.dummy_target_rel_orientation[2],
            self.dummy_target_rel_orientation[3]
        )

        self.publish_markers()

    def publish_goal(self):
        # dummy_targetをgoalとして設定
        self.mand_start_pub.publish(Empty())
        msg = GoalSet()
        msg.drone_id = self.drone_id
        msg.goal = [self.dummy_target_pose.position.x,
                    self.dummy_target_pose.position.y,
                    self.dummy_target_pose.position.z]
        self.goal_pub.publish(msg)

    def publish_escape(self):
        # dummy_targetをescapeとして設定
        self.mand_start_pub.publish(Empty())
        msg = GoalSet()
        msg.drone_id = self.drone_id
        msg.goal = [self.init_pos.x, self.init_pos.y, self.init_pos.z+1.0]
        self.escape_pub.publish(msg)

    def publish_markers(self):
        # route_track_trigger_radiusの球
        route_marker = Marker()
        route_marker.header.frame_id = "world"
        route_marker.header.stamp = rospy.Time.now()
        route_marker.ns = "trigger_spheres"
        route_marker.id = 0
        route_marker.type = Marker.SPHERE
        route_marker.action = Marker.ADD
        route_marker.pose = self.dummy_target_pose
        route_marker.scale = Vector3(self.route_track_trigger_radius*2,
                                     self.route_track_trigger_radius*2,
                                     self.route_track_trigger_radius*2)
        route_marker.color.r = 0.0
        route_marker.color.g = 1.0
        route_marker.color.b = 0.0
        route_marker.color.a = 0.1

        # dummy_flower_trigger_radiusの球
        flower_marker = Marker()
        flower_marker.header.frame_id = "world"
        flower_marker.header.stamp = rospy.Time.now()
        flower_marker.ns = "trigger_spheres"
        flower_marker.id = 1
        flower_marker.type = Marker.SPHERE
        flower_marker.action = Marker.ADD
        flower_marker.pose = self.dummy_target_pose
        flower_marker.scale = Vector3(self.dummy_flower_trigger_radius*2,
                                      self.dummy_flower_trigger_radius*2,
                                      self.dummy_flower_trigger_radius*2)
        flower_marker.color.r = 0.0
        flower_marker.color.g = 1.0
        flower_marker.color.b = 0.0
        flower_marker.color.a = 0.1

        structure_polygon = PolygonStamped()
        structure_polygon.header.frame_id = "world"
        structure_polygon.header.stamp = rospy.Time.now()
        structure_polygon.polygon = Polygon()
        structure_polygon.polygon.points = [
            Point32(x=self.init_pos.x, y=self.init_pos.y, z=self.init_pos.z),
            Point32(x=self.init_pos.x, y=self.init_pos.y, z=self.init_pos.z+1.0),
            Point32(x=self.dummy_target_pose.position.x, y=self.dummy_target_pose.position.y, z=self.dummy_target_pose.position.z),
        ]

        self.structure_polygon_pub.publish(structure_polygon)
        self.marker_pub.publish(route_marker)
        self.marker_pub.publish(flower_marker)

    def noise_process(self):
        # add noise to self.dummy_target_pose
        self.dummy_target_pose.position.x += np.random.normal(0, 0.005)
        self.dummy_target_pose.position.y += np.random.normal(0, 0.005)

        noise_euler = [0.001, 0.001, 0.0]
        quat_cur = [self.dummy_target_pose.orientation.x,
                    self.dummy_target_pose.orientation.y,
                    self.dummy_target_pose.orientation.z,
                    self.dummy_target_pose.orientation.w]
        rot_cur = R.from_quat(quat_cur)
        rot_noise = R.from_euler('xyz', noise_euler)
        rot_new = rot_cur * rot_noise
        quat_new = rot_new.as_quat()
        self.dummy_target_pose.orientation = Quaternion(quat_new[0], quat_new[1], quat_new[2], quat_new[3])

    def process_sequence(self, msg):
        if self.dummy_target_pose is None:
            print("Dummy target not initialized yet.")
            return
        self.noise_process()
        # 現在位置を取得
        current_pos = msg.pose.pose.position
        target_pos = self.dummy_target_pose.position
        target_ori = self.dummy_target_pose.orientation

        self.publish_markers()

        # dummy_targetまでの距離を算出
        dist = math.sqrt((current_pos.x - target_pos.x)**2 + 
                         (current_pos.y - target_pos.y)**2 + 
                         (current_pos.z - target_pos.z)**2)

        # dummy_flower_trigger_radius以内に入ったらTrackingPoseとdummy_target_positionをpublish
        if dist <= self.dummy_flower_trigger_radius:
            # TrackingPose生成
            tp = TrackingPose()
            tp.drone_id = self.drone_id
            tp.distance = self.tracking_distance
            tp.center = Vector3(target_pos.x, target_pos.y, target_pos.z)
            # orientation to normal vector
            rotation = R.from_quat([target_ori.x, target_ori.y, target_ori.z, target_ori.w])
            normal = rotation.apply([1, 0, 0])
            tp.normal = Vector3(normal[0], normal[1], normal[2])

            # print("rotation: ", rotation)
            # print("TrackingPose: ", tp)

            # 状態設定
            # ここではTARGET_STATUS_CAPTUREDを想定
            tp.target_status = TrackingPose.TARGET_STATUS_CAPTURED

            # publish
            self.tracking_pose_pub.publish(tp)

            # dummy_target_positionをpublish
            dummy_pos = PoseStamped()
            dummy_pos.header.frame_id = "world"
            dummy_pos.header.stamp = rospy.Time.now()
            dummy_pos.pose = self.dummy_target_pose
            self.dummy_target_pub.publish(dummy_pos)

class MavrosBridgeClient:
    def __init__(self):
        rospy.init_node('mavros_bridge_client', anonymous=True)
        self.drone_id = rospy.get_param('~self_id', 0)
        self.use_lcm = rospy.get_param('~use_lcm', False)

        # Variables
        self.position_target = np.zeros(3)
        self.init_pose_pid = False
        self.activated = False
        self.mandatory_stop = False
        self.odom_cur = Odometry()
        self.mode = 'setpoint_position'
        self.demo_manager = DemoManager(self.odom_cur, self.drone_id)
        
        # Publishers
        self.setpoint_position_pub = rospy.Publisher('/traj_server/planning/setpoint_position', PositionCommand, queue_size=1)
        # self.setpoint_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.target_marker_pub = rospy.Publisher('/cmd_gcs/setpoint_position_marker', Marker, queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
        self.mand_stop_pub = rospy.Publisher('/mandatory_stop_to_planner', Empty, queue_size=10)

        if self.use_lcm:
            self.takeoff_command_pub = rospy.Publisher('/hardware_bridge/takeoff_mand', CommandTOL, queue_size=1)
            self.land_command_pub = rospy.Publisher('/hardware_bridge/land_mand', CommandTOL, queue_size=1)
            self.odom_sub = rospy.Subscriber('/d2vins/indexed_odometry', IndexedOdometry, self.lcm_odom_callback, queue_size=1)
        else:
            self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
            # self.odom_sub = rospy.Subscriber('/d2vins/odometry', Odometry, self.odom_callback)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

    def timer_callback(self, event):
        if self.mandatory_stop:
            return
        if self.demo_manager is not None and self.odom_cur is not None:
            self.demo_manager.process_sequence(self.odom_cur)

    def lcm_odom_callback(self, msg):
        if msg.drone_id != self.drone_id:
            return
        self.odom_cur = msg

    def odom_callback(self, msg):
        self.odom_cur = msg
        self.demo_manager.transform_by_odom(msg)

    def publish_setpoint(self, dx=0.0, dy=0.0, dz=0.0):
        if self.odom_cur is None:
            print("No odometry data received yet.")
            return

        self.mand_stop_pub.publish(Empty())
        self.mandatory_stop = True

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

        # ---------------------------
        # ローカル座標系 → ワールド座標系 変換
        # ---------------------------
        if self.use_lcm:
            orientation = self.odom_cur.odom.pose.pose.orientation
        else:
            orientation = self.odom_cur.pose.pose.orientation

        q = np.array([orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w])

        # 回転行列
        rot_mat = quaternion_matrix(q)[0:3, 0:3]

        # ローカル → ワールドへの並進ベクトル変換
        local_offset = np.array([dx, dy, dz])
        world_offset = rot_mat.dot(local_offset)

        # ターゲット位置の更新
        self.position_target += world_offset

        # ---------------------------
        # 姿勢を「現在の機体姿勢を維持」する場合
        # ---------------------------
        # そのまま orientation をコピー
        new_orientation = orientation

        # コマンド生成
        if self.mode == 'setpoint_position':
            cmd = PositionCommand()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "world"
            cmd.drone_id = self.drone_id

            cmd.pose.position.x = self.position_target[0]
            cmd.pose.position.y = self.position_target[1]
            cmd.pose.position.z = self.position_target[2]

            # ★ ここで機体の姿勢をそのまま反映
            cmd.pose.orientation.x = new_orientation.x
            cmd.pose.orientation.y = new_orientation.y
            cmd.pose.orientation.z = new_orientation.z
            cmd.pose.orientation.w = new_orientation.w

            self.setpoint_position_pub.publish(cmd)
            # print("Published setpoint: x=%.2f, y=%.2f, z=%.2f" % (
            #     self.position_target[0], 
            #     self.position_target[1], 
            #     self.position_target[2]
            # ))

        else:  # goal mode
            goal_msg = GoalSet()
            goal_msg.drone_id = self.drone_id
            goal_msg.goal = [
                self.position_target[0],
                self.position_target[1],
                self.position_target[2]
            ]
            # goalモードでも姿勢を送りたい場合は独自拡張が必要
            self.goal_pub.publish(goal_msg)
            print("Published goal: x=%.2f, y=%.2f, z=%.2f" % (
                self.position_target[0], 
                self.position_target[1], 
                self.position_target[2]
            ))

        # 可視化用
        self.publish_marker()

    # def publish_setpoint(self, dx=0.0, dy=0.0, dz=0.0):
    #     if self.odom_cur is None:
    #         print("No odometry data received yet.")
    #         return
    #     # if not self.activated and not self.use_lcm:
    #     #     print("Drone not activated yet.")
    #     #     return

    #     self.mand_stop_pub.publish(Empty())

    #     if not self.init_pose_pid:
    #         if self.use_lcm:
    #             self.position_target = np.array([
    #                 self.odom_cur.odom.pose.pose.position.x,
    #                 self.odom_cur.odom.pose.pose.position.y,
    #                 self.odom_cur.odom.pose.pose.position.z
    #             ])
    #         else:
    #             self.position_target = np.array([
    #                 self.odom_cur.pose.pose.position.x,
    #                 self.odom_cur.pose.pose.position.y,
    #                 self.odom_cur.pose.pose.position.z
    #             ])
    #         self.init_pose_pid = True

    #     # Update target position
    #     self.position_target += np.array([dx, dy, dz])

    #     if self.mode == 'setpoint_position':
    #         cmd = PositionCommand()
    #         cmd.header.stamp = rospy.Time.now()
    #         cmd.header.frame_id = "world"
    #         cmd.drone_id = self.drone_id
    #         cmd.pose.position.x = self.position_target[0]
    #         cmd.pose.position.y = self.position_target[1]
    #         cmd.pose.position.z = self.position_target[2]
    #         cmd.pose.orientation.w = 1.0
    #         self.setpoint_position_pub.publish(cmd)
    #         print("Published setpoint: x=%.2f, y=%.2f, z=%.2f" % (
    #             self.position_target[0], 
    #             self.position_target[1], 
    #             self.position_target[2]
    #         ))
    #         # cmd = PoseStamped()
    #         # cmd.header.stamp = rospy.Time.now()
    #         # cmd.header.frame_id = "world"
    #         # cmd.pose.position.x = self.position_target[0]
    #         # cmd.pose.position.y = self.position_target[1]
    #         # cmd.pose.position.z = self.position_target[2]
    #         # cmd.pose.orientation.w = 1.0
    #         # self.setpoint_position_pub.publish(cmd)
    #         # print("Published setpoint: x=%.2f, y=%.2f, z=%.2f" % (
    #         #     self.position_target[0],
    #         #     self.position_target[1],
    #         #     self.position_target[2]
    #         # ))
    #     else:  # goal mode
    #         goal_msg = GoalSet()
    #         goal_msg.drone_id = self.drone_id
    #         goal_msg.goal = [
    #             self.position_target[0],
    #             self.position_target[1],
    #             self.position_target[2]
    #         ]
    #         self.goal_pub.publish(goal_msg)
    #         print("Published goal: x=%.2f, y=%.2f, z=%.2f" % (
    #             self.position_target[0], 
    #             self.position_target[1], 
    #             self.position_target[2]
    #         ))

    #     # Publish visualization marker
    #     self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.5)
        marker.pose.position.x = self.position_target[0]
        marker.pose.position.y = self.position_target[1]
        marker.pose.position.z = self.position_target[2]
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.target_marker_pub.publish(marker)

    def handle_takeoff(self):
        # if self.odom_cur is None:
        #     print("No odometry data received yet.")
        #     return
        # self.demo_manager.fix_target(self.odom_cur)

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

        if self.odom_cur is None:
            print("No odometry data received yet.")
            return
        self.demo_manager.fix_target(self.odom_cur, activate)
        
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
        service_name = '/mavros_bridge/land'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            takeoff_cl = rospy.ServiceProxy(service_name, CommandTOLSrv)
            response = takeoff_cl(altitude=1.0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def call_takeoff_service(self):
        service_name = '/mavros_bridge/takeoff'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            print(f"Service {service_name} is not available. Timeout occurred.")
            return False
        
        try:
            takeoff_cl = rospy.ServiceProxy(service_name, CommandTOLSrv)
            response = takeoff_cl(altitude=1.0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def call_shot_service(self):
        service_name = '/hardware_bridge/rotate_motor'
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
            print("\033[1;32mA: Activate\033[0m, \033[1;32mD: Deactivate\033[0m")
        print("\033[1;32mT: Takeoff\033[0m, \033[1;32mL: Land\033[0m")
        print("\033[1;32mX: Execute Demo Sequence\033[0m, \033[1;32mE: Escape\033[0m, \033[1;32mS: Shot\033[0m")
        print("Arrow keys: Move in x-y plane")
        print("Page Up/Down: Move up/down")
        print("Q: Quit")
        
        while True:
            char = self.getch()
            
            if not self.use_lcm:
                self.init_pose_pid = False
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
                elif char == 'x':
                    if self.demo_manager is None:
                        print("DemoManager not initialized yet.")
                        continue
                    self.mandatory_stop = False
                    self.demo_manager.publish_goal()
                    print("Execute Demo Sequence")
                    print("Publish Approx Goal")
                elif char == 'e':
                    if self.demo_manager is None:
                        print("DemoManager not initialized yet.")
                        continue
                    self.mandatory_stop = False
                    self.demo_manager.publish_escape()
                    print("Escape")

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