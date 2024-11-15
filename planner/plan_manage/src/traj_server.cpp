// traj_server.cpp
#include <plan_manage/traj_server.h>

namespace ego_planner
{

TrajServer::TrajServer(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.param("root_tracking_threshold", root_tracking_threshold_, 0.5);
    nh_.param("update_goal_threshold", update_goal_threshold_, 0.5);
    nh_.param("time_forward", time_forward_, -1.0);
    nh_.param("drone_id", drone_id_, 0);
    nh_.param("update_tracking_goal_threshold", update_tracking_goal_threshold_, 0.5);
    nh_.param<bool>("use_pin_cmd", use_pin_cmd_, false);

    ROS_INFO("[traj_server] root_tracking_threshold: %f", root_tracking_threshold_);
    ROS_INFO("[traj_server] update_goal_threshold: %f", update_goal_threshold_);
    ROS_INFO("[traj_server] time_forward: %f", time_forward_);
    ROS_INFO("[traj_server] drone_id: %d", drone_id_);
    
    heartbeat_time_ = ros::Time(0);
    mode_ = NavigationMode::IDLE;
    resetTraj();

    setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/setpoint_raw", 50);
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/setpoint_position", 50);
    status_pub_ = nh.advertise<swarm_msgs::SystemStatus>("system_status", 50);
    
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 50);
    // fake_pos_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
    // cf_full_state_cmd_pub_ = nh_.advertise<quadrotor_msgs::FullState>("/cmd_full_state", 1);
    // cf_position_cmd_pub_ = nh_.advertise<quadrotor_msgs::Position>("/cmd_position", 1);
    goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 1);
    target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_marker", 1);

    update_mode_srv_ = nh_.advertiseService("/update_mode", &TrajServer::updateModeCallback, this);

    status_sub_ = nh.subscribe("/hardware_bridge/system_status", 1, &TrajServer::statusCallback, this);
    poly_traj_sub_ = nh_.subscribe("planning/trajectory", 1, &TrajServer::polyTrajCallback, this);
    target_pose_sub_ = nh_.subscribe("planning/track_pose", 1, &TrajServer::targetPoseCallback, this);
    setpoint_pos_sub_ = nh_.subscribe("planning/setpoint_position", 1, &TrajServer::setpointPosCallback, this);

    heartbeat_sub_ = nh_.subscribe("heartbeat", 1, &TrajServer::heartbeatCallback, this);
    odom_sub_ = nh_.subscribe("planning/odom", 1, &TrajServer::odomCallback, this);

    cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajServer::cmdTimerCallback, this);
}
}  // namespace ego_planner

using namespace ego_planner;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server");
    // ros::NodeHandle node;
    ros::NodeHandle nh("~");
    TrajServer traj_server(nh);
    ros::Duration(1.0).sleep();
    ROS_INFO("[Traj server]: ready.");
    ros::spin();
    return 0;
}