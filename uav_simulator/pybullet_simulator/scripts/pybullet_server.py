#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from quadrotor_msgs.msg import PositionCommand
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

class CrazyswarmPyBullet:
    def __init__(
            self, 
            num_drones, 
            num_cams,
            init_pos, 
            control_freq_hz=48, 
            camera_freq_hz=12,
            visualize_image=False
            ):
        self.num_drones = num_drones
        self.num_cams = num_cams
        INIT_XYZS = init_pos
        INIT_RPYS = np.array([[0, 0, 0]] * num_drones)
        drone_model = DroneModel.CF2X
        physics = Physics.PYB
        simulation_freq_hz = 240
        self.control_freq_hz = control_freq_hz
        self.camera_freq_hz = camera_freq_hz

        self.env = CtrlAviary(drone_model=drone_model,
                num_drones=num_drones,
                initial_xyzs=INIT_XYZS,
                initial_rpys=INIT_RPYS,
                physics=physics,
                neighbourhood_radius=10,
                pyb_freq=simulation_freq_hz,
                ctrl_freq=control_freq_hz,
                gui=True,
                record=False,
                obstacles=True,
                vision_attributes=True,
                user_debug_gui=False
                )
        obs, _ = self.env.reset()
        # print("obs:", obs) # pos(3),quat(4),rpy(3),vel(3),rpy_rates(3),last_action(4)
        self.cur_state = obs
        self.target_state = obs[:,0:16].copy()
        self.ctrl = [DSLPIDControl(drone_model=drone_model) for _ in range(num_drones)]
        self.visualize_image = visualize_image
        if visualize_image:
            self.window_names = [f"Drone {j+1}" for j in range(num_cams)]
            for name in self.window_names:
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        
        # Publishers for each drone
        self.odom_publishers = [rospy.Publisher(f'drone_{i}_visual_slam/odom', Odometry, queue_size=10) for i in range(num_drones)]
        self.imu_publishers = [rospy.Publisher(f'drone_{i}/imu', Imu, queue_size=10) for i in range(num_drones)]
        self.vision_publishers = [rospy.Publisher(f'drone_{i}/image_raw', Image, queue_size=10) for i in range(num_cams)]

        # Subscribers for each drone
        self.cmd_subscribers = [
            rospy.Subscriber(f'drone_{i}_planning/pos_cmd', PositionCommand, self.cmd_callback, i) for i in range(num_drones)
        ]

        # Timer to call step function at the specified frequency
        rospy.Timer(rospy.Duration(1.0/control_freq_hz), self.ctrl_callback)
        rospy.Timer(rospy.Duration(1.0/camera_freq_hz), self.camera_callback)

    def cmd_callback(self, data, drone_index):
        # Callback to handle fullstate messages
        # rospy.loginfo(f'Received cmdfullstate for drone {drone_index}: {data}')
        self.target_state[drone_index] = np.array([
            data.position.x, data.position.y, data.position.z,
            0, 0, 0, 1,
            0, 0, data.yaw,
            data.velocity.x, data.velocity.y, data.velocity.z,
            0, 0, data.yaw_dot
        ])
        # print("target_pos_d:", self.target_state[0][0:3])
    
    def publish_image(self, image, publisher):
        # Convert image to ROS message and publish
        bridge = CvBridge()
        # print("image:", image.shape)
        bgr_image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        image_message = bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
        image_message.header.stamp = rospy.Time.now()
        image_message.header.frame_id = "world"
        # passthrough to bgr8
        publisher.publish(image_message)

    def pulish_odom(self, state, publisher):
        # Convert state to ROS message and publish
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = state[0]
        odom.pose.pose.position.y = state[1]
        odom.pose.pose.position.z = state[2]
        odom.pose.pose.orientation.x = state[3]
        odom.pose.pose.orientation.y = state[4]
        odom.pose.pose.orientation.z = state[5]
        odom.pose.pose.orientation.w = state[6]
        odom.twist.twist.linear.x = state[10]
        odom.twist.twist.linear.y = state[11]
        odom.twist.twist.linear.z = state[12]
        odom.twist.twist.angular.x = state[13]
        odom.twist.twist.angular.y = state[14]
        odom.twist.twist.angular.z = state[15]
        publisher.publish(odom)

    def publish_imu(self, state, publisher):
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "world"
        imu.orientation.x = state[3]
        imu.orientation.y = state[4]
        imu.orientation.z = state[5]
        imu.orientation.w = state[6]
        imu.angular_velocity.x = state[13]
        imu.angular_velocity.y = state[14]
        imu.angular_velocity.z = state[15]
        imu.linear_acceleration.x = state[10]
        imu.linear_acceleration.y = state[11]
        imu.linear_acceleration.z = state[12]
        publisher.publish(imu)
    
    def ctrl_callback(self, event):
        start_time = rospy.get_time()
        action = np.zeros((self.num_drones, 4))
        for i in range(self.num_drones):
            # print("agent:", i, "target_pos:", self.target_state[i])
            # target_pos_debug = self.cur_state[i,0:3]+[0.5,0,-0.1]
            # print("target_pos_debug:", target_pos_debug)
            
            action[i,:], _, _ = self.ctrl[i].computeControlFromState(
                control_timestep=self.env.CTRL_TIMESTEP,
                state=self.cur_state[i],
                # target_pos=target_pos_debug,
                target_pos=self.target_state[i][0:3],
                target_rpy=self.target_state[i][7:10],
                target_vel=self.target_state[i][10:13],
                # target_rpy_rates=self.target_state[i][13:16]
            )
        obs, _, _, _, _ = self.env.step(action)

        # Timer callback to call the step function regularly
        for i in range(self.num_drones):
            self.cur_state[i] = obs[i]
            self.pulish_odom(self.cur_state[i], self.odom_publishers[i])
            self.publish_imu(self.cur_state[i], self.imu_publishers[i])
        step_time = rospy.get_time() - start_time
        # print("Ctrl Maximum Frequency:", 1.0/step_time)

    def camera_callback(self, event):
        # Timer callback to call the step function regularly
        start_time = rospy.get_time()
        for i in range(self.num_cams):
            image, _, _ = self.env._getDroneImages(i)
            self.publish_image(image, self.vision_publishers[i])
            # if self.visualize_image:
            #     cv2.imshow(self.window_names[i], image)
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         break
        step_time = rospy.get_time() - start_time
        print("Camera Maximum Frequency:", 1.0/step_time)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('drone_simulator_ros_wrapper')
        num_drones = rospy.get_param('~num_drones', 1)
        num_cams = rospy.get_param('~num_cams', 1)
        control_freq_hz = rospy.get_param('~control_freq_hz', 240)
        camera_freq_hz = rospy.get_param('~camera_freq_hz', 10)
        visualize_image = rospy.get_param('~visualize_image', False)
        config_file = rospy.get_param('~config_file', None)

        # yamlファイルから初期位置を読み込む
        if config_file is not None:
            with open(config_file) as file:
                config = yaml.safe_load(file)
            num_drones = config['drones']['num_drones']
            num_cams = config['drones']['num_cams']
            control_freq_hz = config['drones']['control_freq_hz']
            camera_freq_hz = config['drones']['camera_freq_hz']
            visualize_image = config['drones']['visualize_image']
            init_pos = np.zeros((num_drones, 3))
            for i in range(num_drones):
                pos = config['drones']['drone_'+str(i)]['initial_position']
                init_pos[i,0] = pos['x']
                init_pos[i,1] = pos['y']
                init_pos[i,2] = pos['z']
        else:
            init_pos = np.array([[0, 0, 1.5]] * num_drones)

        wrapper = CrazyswarmPyBullet(
            num_drones, 
            num_cams,
            init_pos, 
            control_freq_hz, 
            camera_freq_hz,
            visualize_image)
        wrapper.run()
    except rospy.ROSInterruptException:
        pass