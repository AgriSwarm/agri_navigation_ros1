#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <swarm_msgs/SystemStatus.h>
#include <std_srvs/SetBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <tf/transform_broadcaster.h>
#include <hardware_utils/RotateMotor.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


tf::TransformBroadcaster* br_ptr;
ros::Subscriber _cmd_sub, _status_sub;
ros::Publisher  _odom_1_pub, _odom_2_pub, status_pub_, pict_state_pub_, debug_text_pub_;
ros::Timer pict_state_timer_;
swarm_msgs::SystemStatus status_cur_;
ros::ServiceServer _takeoff_server, _activate_server, _shot_server;
ros::Publisher shot_cone_pub_;

quadrotor_msgs::PositionCommand _cmd;
double _init_x, _init_y, _init_z, _init_yaw;
int _drone_id;

bool rcv_cmd = false;


void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd)
{	
	rcv_cmd = true;
	_cmd    = cmd;
}

bool activateCallback(std_srvs::SetBool::Request &req,
                     std_srvs::SetBool::Response &res)
{
    ROS_INFO("Activate service called with data: %s", req.data ? "true" : "false");
    
	status_cur_.ap_status = req.data ? swarm_msgs::SystemStatus::AP_GUIDED : swarm_msgs::SystemStatus::AP_STABILIZE;
	status_cur_.infra_status = req.data ? swarm_msgs::SystemStatus::INFRA_ARMED : swarm_msgs::SystemStatus::INFRA_INACTIVE;

	res.success = true;
    res.message = "Activation status changed";
    return true;
}

bool takeoffCallback(mavros_msgs::CommandTOL::Request &req,
                    mavros_msgs::CommandTOL::Response &res)
{
    ROS_INFO("Takeoff service called");
	ROS_INFO("Takeoff to altitude: %f", req.altitude);
	
	_cmd.position.x = _init_x;
	_cmd.position.y = _init_y;
	_cmd.position.z = _init_z + req.altitude;
    _cmd.yaw = _init_yaw;
	rcv_cmd = true;

    res.success = true;

    return true;
}

void statusCallback(const swarm_msgs::SystemStatus msg)
{
    if(static_cast<int>(msg.drone_id) != _drone_id){
        return;
    }
    status_cur_.nav_status = msg.nav_status;
}

void pubOdom()
{	
	nav_msgs::Odometry odom;
	odom.header.stamp    = ros::Time::now();
	odom.header.frame_id = "world";

	if(rcv_cmd)
	{
	    odom.pose.pose.position.x = _cmd.position.x;
	    odom.pose.pose.position.y = _cmd.position.y;
	    odom.pose.pose.position.z = _cmd.position.z;

		Eigen::Vector3d alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
		Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
		Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
		Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
		Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
		Eigen::Vector3d zB = xB.cross(yB);
		Eigen::Matrix3d R;
		R.col(0) = xB;
		R.col(1) = yB;
		R.col(2) = zB;
		Eigen::Quaterniond q(R);
	    odom.pose.pose.orientation.w = q.w();
	    odom.pose.pose.orientation.x = q.x();
	    odom.pose.pose.orientation.y = q.y();
	    odom.pose.pose.orientation.z = q.z();

	    odom.twist.twist.linear.x = _cmd.velocity.x;
	    odom.twist.twist.linear.y = _cmd.velocity.y;
	    odom.twist.twist.linear.z = _cmd.velocity.z;

	    odom.twist.twist.angular.x = _cmd.acceleration.x;
	    odom.twist.twist.angular.y = _cmd.acceleration.y;
	    odom.twist.twist.angular.z = _cmd.acceleration.z;

		// tfブロードキャストを追加
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(_cmd.position.x, _cmd.position.y, _cmd.position.z));
        tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
        transform.setRotation(q_tf);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
		if(br_ptr != nullptr) {
			br_ptr->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
		}
	}
	else
	{
        odom.pose.pose.position.x = _init_x;
        odom.pose.pose.position.y = _init_y;
        odom.pose.pose.position.z = _init_z;

        // yawからクォータニオンを計算
        tf::Quaternion q;
        q.setRPY(0, 0, _init_yaw);  // Roll=0, Pitch=0, Yaw=_init_yaw

        // クォータニオンをodomにセット
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        // tfブロードキャストを追加
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(_init_x, _init_y, _init_z));
        transform.setRotation(q);  // 同じクォータニオンを使用
        
        if(br_ptr != nullptr) {
            br_ptr->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
        }
    }

    _odom_1_pub.publish(odom);
	_odom_2_pub.publish(odom);
}

void pubDebugText(swarm_msgs::SystemStatus msg)
{
    jsk_rviz_plugins::OverlayText text;
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.width = 500;
    text.height = 500;
    text.left = 10;
    text.top = 10;
    text.bg_color.r = 0.0;
    text.bg_color.g = 0.0;
    text.bg_color.b = 0.0;
    text.bg_color.a = 0.5;
    text.line_width = 2;
    text.text_size = 20;
    text.font = "DejaVu Sans Mono";
    text.fg_color.r = 1.0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;

    std::string infra_status;
    switch(msg.infra_status) {
        case swarm_msgs::SystemStatus::INFRA_INACTIVE:
            infra_status = "INACTIVE";
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_CONNECTED:
            infra_status = "AP_CONNECTED";
            break;
        case swarm_msgs::SystemStatus::INFRA_ODOM_READY:
            infra_status = "ODOM_READY";
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_EKF_READY:
            infra_status = "AP_EKF_READY";
            break;
        case swarm_msgs::SystemStatus::INFRA_ARMED:
            infra_status = "ARMED";
            break;
    }

    // std::string status_str = msg.ap_status + "::" + msg.nav_status;
    std::string status_str = "INFRA: " + infra_status + "\nAP: " + msg.ap_status + "\nNAV: " + msg.nav_status;
    text.text = status_str;

    debug_text_pub_.publish(text);
}

void pubPictgramState(swarm_msgs::SystemStatus msg)
{
    jsk_rviz_plugins::PictogramArray pictogram_array;
    pictogram_array.header = msg.header;
    pictogram_array.header.frame_id = "base_link";

    // ステータス文字列の結合と分割
    std::string status_str = msg.ap_status + "::" + msg.nav_status;
    std::vector<std::string> chunks;
    for (size_t i = 0; i < status_str.length(); i += 2) {
        if (i + 1 < status_str.length()) {
            chunks.push_back(status_str.substr(i, 2));
        } else {
            chunks.push_back(status_str.substr(i, 1) + " ");
        }
    }

    // 各文字のPictogramを作成
    float horizontal_offset = -0.02f * (chunks.size() - 1) / 2.0f;
    for (size_t i = 0; i < chunks.size(); i++) {
        jsk_rviz_plugins::Pictogram status_pic;
        status_pic.header = msg.header;
        status_pic.header.frame_id = "base_link";
        status_pic.pose.position.x = horizontal_offset + 0.02f * i;
        status_pic.pose.position.y = 0.0;
        status_pic.pose.position.z = 0.15; // Offset for upper text
        status_pic.pose.orientation.x = 0.0;
        status_pic.pose.orientation.y = -0.7;
        status_pic.pose.orientation.z = 0.0;
        status_pic.pose.orientation.w = 0.7;
        status_pic.mode = jsk_rviz_plugins::Pictogram::STRING_MODE;
        status_pic.character = chunks[chunks.size() - i - 1];
        status_pic.size = 0.05;
        status_pic.ttl = 0.0;
        status_pic.action = jsk_rviz_plugins::Pictogram::ADD;

        // Set color based on status
        status_pic.color.r = 25 / 255.0;
        status_pic.color.g = 255 / 255.0;
        status_pic.color.b = 240 / 255.0;
        status_pic.color.a = 1.0;

        pictogram_array.pictograms.push_back(status_pic);
    }

    // Infra status pictogram
    jsk_rviz_plugins::Pictogram infra_pic;
    infra_pic.header = msg.header;
    infra_pic.header.frame_id = "base_link";
    infra_pic.pose.position.x = 0.0;
    infra_pic.pose.position.y = 0.0;
    infra_pic.pose.position.z = 0.1; // Offset for lower icon
    infra_pic.pose.orientation.x = 0.0;
    infra_pic.pose.orientation.y = -0.7;
    infra_pic.pose.orientation.z = 0.0;
    infra_pic.pose.orientation.w = 0.7;
    infra_pic.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
    
    // Set pictogram character based on infra_status
    switch(msg.infra_status) {
        case swarm_msgs::SystemStatus::INFRA_INACTIVE:
            infra_pic.character = "three-dots";
            infra_pic.color.r = 150 / 255.0;
            infra_pic.color.g = 150 / 255.0;
            infra_pic.color.b = 150 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_CONNECTED:
            infra_pic.character = "fa-link";
            infra_pic.color.r = 150 / 255.0;
            infra_pic.color.g = 150 / 255.0;
            infra_pic.color.b = 150 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_ODOM_READY:
            infra_pic.character = "fa-mendeley";
            infra_pic.color.r = 150 / 255.0;
            infra_pic.color.g = 150 / 255.0;
            infra_pic.color.b = 150 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_EKF_READY:
            infra_pic.character = "fa-mendeley";
            infra_pic.color.r = 25 / 255.0;
            infra_pic.color.g = 255 / 255.0;
            infra_pic.color.b = 240 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_ARMED:
            infra_pic.character = "fa-rocket";
            infra_pic.color.r = 25 / 255.0;
            infra_pic.color.g = 255 / 255.0;
            infra_pic.color.b = 240 / 255.0;
            break;
    }
    
    infra_pic.color.a = 1.0;
    infra_pic.size = 0.1;
    infra_pic.ttl = 0.0;
    infra_pic.action = jsk_rviz_plugins::Pictogram::ADD;

    pictogram_array.pictograms.push_back(infra_pic);

    pict_state_pub_.publish(pictogram_array);
}


void pictStateTimerCallback(const ros::TimerEvent&)
{
    pubDebugText(status_cur_);
    pubPictgramState(status_cur_);
    status_pub_.publish(status_cur_);
}

void pubShotCone(const std_msgs::ColorRGBA &color, double duration)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shot_cone";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(duration);
    // marker.lifetime = ros::Duration(0.0);  // 一度だけ表示

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 円錐のパラメータ設定
    double length = 0.5;            // 円錐の高さ
    double coneAngle = 0.4* M_PI / 2;  // 全広がり角 (270°)
    double halfAngle = coneAngle / 2.0;
    double radius = length * fabs(tan(halfAngle));
    int segments = 50;              // 底面円の分割数

    // 先端（tip）は原点
    geometry_msgs::Point tip;
    tip.x = 0.1;
    tip.y = 0.0;
    tip.z = 0.0;

    // 底面円上の各点を計算（x = length 平面上）
    std::vector<geometry_msgs::Point> base_points;
    for (int i = 0; i < segments; ++i)
    {
        double theta = 2 * M_PI * i / segments;
        geometry_msgs::Point p;
        p.x = length;
        p.y = radius * cos(theta);
        p.z = radius * sin(theta);
        base_points.push_back(p);
    }

    // 円錐の側面（三角形リスト）を生成（表面）
    for (int i = 0; i < segments; ++i)
    {
        int next = (i + 1) % segments;
        marker.points.push_back(tip);
        marker.points.push_back(base_points[i]);
        marker.points.push_back(base_points[next]);
    }
    // 裏面も描画するため、頂点順序を反転した三角形を追加
    for (int i = 0; i < segments; ++i)
    {
        int next = (i + 1) % segments;
        marker.points.push_back(tip);
        marker.points.push_back(base_points[next]);
        marker.points.push_back(base_points[i]);
    }

    // 各頂点に個別の色を設定せず、マーカー全体の色を設定する（透過効果を正しく反映させるため）
    marker.color = color;

    shot_cone_pub_.publish(marker);
}

bool shotConeCallback(hardware_utils::RotateMotor::Request &req,
                      hardware_utils::RotateMotor::Response &res)
{
    std_msgs::ColorRGBA g_cone_color;
    g_cone_color.r = 1.0;
    g_cone_color.g = 1.0;
    g_cone_color.b = 0.0;
    g_cone_color.a = 0.2;
    pubShotCone(g_cone_color, req.duration);
    res.success = true;
    res.message = "Cone shot!";
    return true;
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh("~");

	br_ptr = new tf::TransformBroadcaster();

    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);
    nh.param("init_yaw", _init_yaw, 0.0);
	nh.param("drone_id", _drone_id, 0);

    _odom_1_pub = nh.advertise<nav_msgs::Odometry>("odometry_1", 1);               
	_odom_2_pub = nh.advertise<nav_msgs::Odometry>("odometry_2", 1); 
	status_pub_ = nh.advertise<swarm_msgs::SystemStatus>("/hardware_bridge/system_status", 10);  
	pict_state_pub_ = nh.advertise<jsk_rviz_plugins::PictogramArray>("/hardware_bridge/system_status_pict", 10);       
    debug_text_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/hardware_bridge/system_status_text", 10);
    shot_cone_pub_ = nh.advertise<visualization_msgs::Marker>("/hardware_bridge/shot_cone", 1);
	  
	pict_state_timer_ = nh.createTimer(ros::Duration(1.0), pictStateTimerCallback);

	_takeoff_server = nh.advertiseService("/mavros_bridge/takeoff", takeoffCallback);
    _activate_server = nh.advertiseService("/mavros_bridge/activate", activateCallback);
    _shot_server = nh.advertiseService("/hardware_bridge/rotate_motor", shotConeCallback);

	_cmd_sub  = nh.subscribe( "command", 1, rcvPosCmdCallBack );
	_status_sub = nh.subscribe("/traj_server/system_status", 1, statusCallback);

	status_cur_.drone_id = _drone_id;
	status_cur_.infra_status = swarm_msgs::SystemStatus::INFRA_INACTIVE;
	status_cur_.ap_status = swarm_msgs::SystemStatus::AP_STABILIZE;
	status_cur_.nav_status = swarm_msgs::SystemStatus::NAV_IDLE;

    ros::Rate rate(10);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

	delete br_ptr;
    return 0;
}