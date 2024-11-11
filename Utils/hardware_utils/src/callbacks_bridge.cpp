#include "hard_mavros_bridge.h"

void MavrosBridge::stateCallback(mavros_msgs::State msg)
{
    last_state_ = msg;
    ap_connected_ = msg.connected;
    // pubPictgramState(msg);

    status_cur_.infra_status = swarm_msgs::SystemStatus::INFRA_INACTIVE;
    if(msg.connected){
        status_cur_.infra_status = swarm_msgs::SystemStatus::INFRA_AP_CONNECTED;
    }
    if(nav_initialized_){
        status_cur_.infra_status = swarm_msgs::SystemStatus::INFRA_ODOM_READY;
    }
    if(msg.armed){
        status_cur_.infra_status = swarm_msgs::SystemStatus::INFRA_ARMED;
    }
    status_cur_.ap_status = msg.mode;

    if (!ap_initialized_ && nav_initialized_)
    {
        initialSetup();
        ap_initialized_ = true;
    }
}

void MavrosBridge::poseImuCallback(const geometry_msgs::PoseStampedConstPtr& pose, const sensor_msgs::ImuConstPtr& imu)
{
    nav_msgs::Odometry odom;
    odom.header = pose->header;
    odom.pose.pose.position = pose->pose.position;
    odom.pose.pose.orientation = imu->orientation;
    odom.twist.twist.linear.x = imu->linear_acceleration.x;
    odom.twist.twist.linear.y = imu->linear_acceleration.y;
    odom.twist.twist.linear.z = imu->linear_acceleration.z;
    odom.twist.twist.angular.x = imu->angular_velocity.x;
    odom.twist.twist.angular.y = imu->angular_velocity.y;
    odom.twist.twist.angular.z = imu->angular_velocity.z;
    odom_pub_.publish(odom);
}

void MavrosBridge::statusCallback(const swarm_msgs::SystemStatus msg)
{
    if(static_cast<int>(msg.drone_id) != self_id){
        return;
    }
    status_cur_.nav_status = msg.nav_status;
}

void MavrosBridge::thermalTimerCallback(const ros::TimerEvent&)
{
    publishTemp("/sys/devices/virtual/thermal/thermal_zone0/temp", pub_temp0_);
    publishTemp("/sys/devices/virtual/thermal/thermal_zone1/temp", pub_temp1_);
}

void MavrosBridge::rcCallback(mavros_msgs::RCIn msg)
{
    sensor_msgs::Joy joy = convertRCtoJoy(msg);
    joy_pub_.publish(joy);
}

bool MavrosBridge::activateCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    ROS_INFO("activate %u", req.data);
    res.success = activate(req.data);
    return true;
}

void MavrosBridge::takeoffMandCallback(swarm_msgs::CommandTOL msg)
{
    if(msg.drone_id != self_id){
        return;
    }
    if(activate(true)){
        mavros_msgs::CommandTOL mavros_msg;
        mavros_msg.request.altitude = msg.altitude;
        mavros_msg.request.latitude = msg.latitude;
        mavros_msg.request.longitude = msg.longitude;
        mavros_msg.request.min_pitch = msg.min_pitch;
        mavros_msg.request.yaw = msg.yaw;
        ros::Duration(1.0).sleep();
        if (takeoff_client_.call(mavros_msg)){
            ROS_INFO("Takeoff command sent");
        }else{
            ROS_ERROR("Failed to send takeoff command");
        }
    }else{
        ROS_ERROR("Failed to activate");
    }
}

void MavrosBridge::landMandCallback(swarm_msgs::CommandTOL msg)
{
    if(msg.drone_id != self_id){
        return;
    }
    mavros_msgs::CommandTOL mavros_msg;
    mavros_msg.request.altitude = msg.altitude;
    mavros_msg.request.latitude = msg.latitude;
    mavros_msg.request.longitude = msg.longitude;
    mavros_msg.request.min_pitch = msg.min_pitch;
    mavros_msg.request.yaw = msg.yaw;

    if(land_client_.call(mavros_msg)){
        ROS_INFO("Land command sent");
    }else{
        ROS_ERROR("Failed to send land command");
    }
}

void MavrosBridge::pictStateTimerCallback(const ros::TimerEvent&)
{
    if(!nav_initialized_){
        transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "base_link"));
    }
    pubPictgramState(status_cur_);
    status_pub_.publish(status_cur_);
}

void MavrosBridge::batteryCallback(sensor_msgs::BatteryState msg)
{
    float voltage = msg.voltage;
    float min_voltage = 3.4 * cells_batt_;
    float max_voltage = 4.2 * cells_batt_;
    if (voltage < min_voltage){
        voltage = min_voltage;
    }
    if (voltage > max_voltage){
        voltage = max_voltage;
    }
    int battery = (voltage - min_voltage) / (max_voltage - min_voltage) * 100;
    std_msgs::Float32 battery_msg;
    battery_msg.data = battery;
    battery_pub_.publish(battery_msg);
}

void MavrosBridge::odomCallback(nav_msgs::Odometry msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    odom_cur_ = msg;
    // vision_pose_pub_.publish(pose);

    if(!nav_initialized_){
        ROS_INFO("Nav initialized");
        nav_initialized_ = true;
    }
}

void MavrosBridge::visionPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!nav_initialized_){
        ROS_INFO("Nav initialized");
        nav_initialized_ = true;
    }
}

void MavrosBridge::APEKFPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(status_cur_.infra_status < swarm_msgs::SystemStatus::INFRA_AP_EKF_READY){
        status_cur_.infra_status = swarm_msgs::SystemStatus::INFRA_AP_EKF_READY;
    }
}