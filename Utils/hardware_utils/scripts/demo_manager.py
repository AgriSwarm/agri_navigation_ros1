#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from quadrotor_msgs.msg import TrackingPose, GoalSet

class DemoManager:
    def __init__(self, init_odom, drone_id):
        # パラメータ設定
        self.dummy_flower_trigger_radius = 1.0
        self.route_track_trigger_radius = 0.5
        self.dummy_target_rel_position = [3.0, 3.0, 1.0]
        self.dummy_target_rel_orientation = [0.0, 0.0, 0.0, 1.0]
        self.drone_id = drone_id

        init_pos = init_odom.pose.pose.position
        init_ori = init_odom.pose.pose.orientation
        # 相対位置を加算
        dummy_x = init_pos.x + self.dummy_target_rel_position[0]
        dummy_y = init_pos.y + self.dummy_target_rel_position[1]
        dummy_z = init_pos.z + self.dummy_target_rel_position[2]

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

        self.goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
        self.tracking_pose_pub = rospy.Publisher("/traj_server/planning/track_pose", TrackingPose, queue_size=10)
        self.marker_pub = rospy.Publisher("flower_detection_spheres", Marker, queue_size=10)
        self.dummy_target_pub = rospy.Publisher("dummy_target_position", Point, queue_size=10)

    def publish_goal(self):
        # dummy_targetをgoalとして設定
        msg = GoalSet()
        msg.drone_id = self.drone_id
        msg.goal = [self.dummy_target_pose.position.x,
                    self.dummy_target_pose.position.y,
                    self.dummy_target_pose.position.z]
        self.goal_pub.publish(msg)

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
        route_marker.color.a = 0.5

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
        flower_marker.color.r = 1.0
        flower_marker.color.g = 0.0
        flower_marker.color.b = 0.0
        flower_marker.color.a = 0.5

        self.marker_pub.publish(route_marker)
        self.marker_pub.publish(flower_marker)

    def process_sequence(self, msg):
        # 現在位置を取得
        current_pos = msg.pose.pose.position
        target_pos = self.dummy_target_pose.position

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
            tp.distance = dist
            tp.center = Vector3(target_pos.x, target_pos.y, target_pos.z)
            tp.normal = Vector3(0.0, 0.0, 1.0)  # 適当な法線ベクトル
            # 状態設定
            # ここではTARGET_STATUS_CAPTUREDを想定
            tp.target_status = TrackingPose.TARGET_STATUS_CAPTURED

            # publish
            self.tracking_pose_pub.publish(tp)

            # dummy_target_positionをpublish
            dummy_pos = Point(target_pos.x, target_pos.y, target_pos.z)
            self.dummy_target_pub.publish(dummy_pos)