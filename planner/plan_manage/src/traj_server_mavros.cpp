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
    mode_ = NavigationMode::INACTIVE;
    resetTraj();

    pos_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/position_cmd", 50);
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 50);
    // fake_pos_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
    // cf_full_state_cmd_pub_ = nh_.advertise<quadrotor_msgs::FullState>("/cmd_full_state", 1);
    // cf_position_cmd_pub_ = nh_.advertise<quadrotor_msgs::Position>("/cmd_position", 1);
    goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 1);
    target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_marker", 1);

    update_mode_srv_ = nh_.advertiseService("/update_mode", &TrajServer::updateModeCallback, this);

    poly_traj_sub_ = nh_.subscribe("planning/trajectory", 1, &TrajServer::polyTrajCallback, this);
    target_pose_sub_ = nh_.subscribe("planning/track_pose", 1, &TrajServer::targetPoseCallback, this);
    heartbeat_sub_ = nh_.subscribe("heartbeat", 1, &TrajServer::heartbeatCallback, this);
    odom_sub_ = nh_.subscribe("planning/odom", 1, &TrajServer::odomCallback, this);

    cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajServer::cmdTimerCallback, this);
}

void TrajServer::resetTraj()
{
    traj_.reset(new poly_traj::Trajectory());
    traj_duration_ = 0.0;
    traj_start_time_ = ros::Time(0);
    traj_id_ = -1;
    first_sensing_ = true;
}

bool TrajServer::updateModeCallback(quadrotor_msgs::UpdateMode::Request& req,
                        quadrotor_msgs::UpdateMode::Response& res)
{
    if (req.mode == quadrotor_msgs::UpdateMode::Request::INACTIVE)
    {
        updateMode(NavigationMode::INACTIVE);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::SEARCH)
    {
        updateMode(NavigationMode::SEARCH);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::APPROACH)
    {
        updateMode(NavigationMode::APPROACH);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::ROOT_TRACKING)
    {
        updateMode(NavigationMode::ROOT_TRACKING);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::PURE_TRACKING)
    {
        updateMode(NavigationMode::PURE_TRACKING);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::HOVERING)
    {
        updateMode(NavigationMode::HOVERING);
    }
    else
    {
        ROS_ERROR("[traj_server] Unrecognized mode!");
    }
    res.success = true;
    return true;
}

void TrajServer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_state_.pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_state_.vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    odom_state_.yaw = tf::getYaw(q);
    odom_received_ = true;
}

void TrajServer::heartbeatCallback(const std_msgs::Empty::ConstPtr &msg)
{
    heartbeat_time_ = ros::Time::now();
}

void TrajServer::polyTrajCallback(const traj_utils::PolyTraj::ConstPtr &msg)
{
    if (msg->order != 5)
    {
        ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
        return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
        ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
        return;
    }

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
        int i6 = i * 6;
        cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
            msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
        cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
            msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
        cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
            msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

        dura[i] = msg->duration[i];
    }

    traj_.reset(new poly_traj::Trajectory(dura, cMats));

    traj_start_time_ = msg->start_time;
    traj_duration_ = traj_->getTotalDuration();
    traj_id_ = msg->traj_id;

    if (mode_ == NavigationMode::INACTIVE){
        updateMode(NavigationMode::SEARCH);
    }
}

void TrajServer::publishGoal(const Eigen::Vector3d &goal_pos)
{
    quadrotor_msgs::GoalSet goal;
    goal.drone_id = drone_id_;
    goal.goal[0] = goal_pos(0);
    goal.goal[1] = goal_pos(1);
    goal.goal[2] = goal_pos(2);
    goal_pub_.publish(goal);
}

void TrajServer::publishFakeCmd(const DroneState &state)
{
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;
    
    cmd.position.x = state.pos(0);
    cmd.position.y = state.pos(1);
    cmd.position.z = state.pos(2);
    cmd.yaw = state.yaw;

    if(state.only_pose){
        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.velocity.z = 0;
        cmd.acceleration.x = 0;
        cmd.acceleration.y = 0;
        cmd.acceleration.z = 0;
        cmd.jerk.x = 0;
        cmd.jerk.y = 0;
        cmd.jerk.z = 0;
        cmd.yaw_dot = 0;
    }else{
        cmd.velocity.x = state.vel(0);
        cmd.velocity.y = state.vel(1);
        cmd.velocity.z = state.vel(2);
        cmd.acceleration.x = state.acc(0);
        cmd.acceleration.y = state.acc(1);
        cmd.acceleration.z = state.acc(2);
        cmd.jerk.x = state.jerk(0);
        cmd.jerk.y = state.jerk(1);
        cmd.jerk.z = state.jerk(2);
        cmd.yaw_dot = state.yaw_rate;
    }

    fake_pos_cmd_pub_.publish(cmd);
}

void TrajServer::publishCFCmd(const DroneState &state)
{
    quadrotor_msgs::FullState cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.pose.position.x = state.pos(0);
    cmd.pose.position.y = state.pos(1);
    cmd.pose.position.z = state.pos(2);
    Eigen::Quaterniond q(Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()));
    cmd.pose.orientation.x = q.x();
    cmd.pose.orientation.y = q.y();
    cmd.pose.orientation.z = q.z();
    cmd.pose.orientation.w = q.w();
    cmd.twist.linear.x = state.vel(0);
    cmd.twist.linear.y = state.vel(1);
    cmd.twist.linear.z = state.vel(2);
    cmd.twist.angular.x = 0;
    cmd.twist.angular.y = 0;
    cmd.twist.angular.z = state.yaw_rate;
    cmd.acc.x = state.acc(0);
    cmd.acc.y = state.acc(1);
    cmd.acc.z = state.acc(2);
    cf_full_state_cmd_pub_.publish(cmd);
}

void TrajServer::publishCFPositionCmd(const DroneState &state)
{
    quadrotor_msgs::Position cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.x = state.pos(0);
    cmd.y = state.pos(1);
    cmd.z = state.pos(2);
    cmd.yaw = state.yaw;
    cf_position_cmd_pub_.publish(cmd);
}

void TrajServer::publishMavrosCmd(const DroneState &state)
{
    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    cmd.position.x = state.pos(0);
    cmd.position.y = state.pos(1);
    cmd.position.z = state.pos(2);
    cmd.velocity.x = state.vel(0);
    cmd.velocity.y = state.vel(1);
    cmd.velocity.z = state.vel(2);
    cmd.acceleration_or_force.x = state.acc(0);
    cmd.acceleration_or_force.y = state.acc(1);
    cmd.acceleration_or_force.z = state.acc(2);
    cmd.yaw = state.yaw;
    cmd.yaw_rate = state.yaw_rate;
    pos_cmd_pub.publish(cmd);

    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "base_link";
    twist.twist.angular.x = 0;
    twist.twist.angular.y = 0;
    twist.twist.angular.z = state.yaw_rate;
    twist.twist.linear.x = state.vel(0);
    twist.twist.linear.y = state.vel(1);
    twist.twist.linear.z = state.vel(2);
    twist_pub.publish(twist);
}

void TrajServer::publishCmd(const DroneState &state)
{
    if(state.pos.norm() < 1e-6 && state.vel.norm() < 1e-6 && state.acc.norm() < 1e-6)
    {
        ROS_ERROR("[traj_server] Invalid state command!");
        return;
    }
    // if(state.only_pose){
    //     publishCFPositionCmd(state);
    // }else{
    //     publishCFCmd(state);
    // }
    publishMavrosCmd(state);

    // publishFakeCmd(state);
    last_cmd_state_ = state;
    cmd_received_ = true;
}

void TrajServer::publishPinCmd()
{
    if(cmd_received_)
    {
        DroneState state;
        state.pos = last_cmd_state_.pos;
        state.yaw = last_cmd_state_.yaw;
        state.only_pose = true;
        publishCmd(state);
    }else if(odom_received_)
    {
        publishHoverCmd();
    }
    else
    {
        ROS_ERROR("[traj_server] Not found last state");
    }
}

void TrajServer::publishHoverCmd()
{
    if(!odom_received_)
    {
        ROS_ERROR("[traj_server] Not found odom state");
        return;
    }
    DroneState state;
    state.pos = odom_state_.pos;      
    state.yaw = odom_state_.yaw;
    state.only_pose = true;
    publishCmd(state);
}

void TrajServer::targetPoseCallback(const quadrotor_msgs::TrackingPose::ConstPtr &msg)
{
    if(msg->target_status == quadrotor_msgs::TrackingPose::TARGET_STATUS_APPROXIMATE)
    {
        Eigen::Vector3d goal_pos = Eigen::Vector3d(msg->center.x, msg->center.y, msg->center.z);
        Eigen::Vector3d diff = goal_pos - last_goal_pos_;
        if(diff.norm() > update_goal_threshold_ || first_sensing_)
        {
            publishGoal(goal_pos);
            last_goal_pos_ = goal_pos;
            first_sensing_ = false;
        }
        updateMode(NavigationMode::APPROACH);
    }
    else if(msg->target_status == quadrotor_msgs::TrackingPose::TARGET_STATUS_CAPTURED)
    {
        tracking_state_ = computeTrackingState(msg);
        visualizeTarget(tracking_state_.pos);
        Eigen::Vector3d diff = tracking_state_.pos - last_cmd_state_.pos;
        // ROS_INFO("[traj_server] diff: %f", diff.norm());
        if(diff.norm() > root_tracking_threshold_)
        {
            updateMode(NavigationMode::ROOT_TRACKING);
            executeRootTracking(tracking_state_);
        }
        else
        {
            updateMode(NavigationMode::PURE_TRACKING);
        }
    }
    else if(msg->target_status == quadrotor_msgs::TrackingPose::TARGET_STATUS_LOST)
    {
        ROS_INFO("[traj_server] Mode changed from %s to SEARCH", modeToString(mode_).c_str());
        updateMode(NavigationMode::SEARCH);
        resetTraj();
    }
    else
    {
        ROS_ERROR("[traj_server] Unrecognized target status!");
        updateMode(NavigationMode::INACTIVE);
    }
}

void TrajServer::executeRootTracking(DroneState target_state)
{
    // TODO: implement pose diff evaluation
    Eigen::Vector3d diff = target_state.pos - last_tracking_goal_.pos;
    if(diff.norm() > update_tracking_goal_threshold_ || first_root_tracking_)
    {
        publishGoal(target_state.pos);
        last_tracking_goal_ = target_state;
        first_root_tracking_ = false;
    }
}

void TrajServer::updateMode(NavigationMode mode)
{
    if(mode_ == mode)
    {
        return;
    }
    // AnyMode -> ROOT_TRACKING
    if(mode == NavigationMode::ROOT_TRACKING){
        first_root_tracking_ = true;
    }
    ROS_INFO("[traj_server] Mode changed from %s to %s", modeToString(mode_).c_str(), modeToString(mode).c_str());
    mode_ = mode;
}

DroneState TrajServer::computeTrackingState(const quadrotor_msgs::TrackingPose::ConstPtr &msg)
{
    DroneState state;
    float distance = msg->distance;
    geometry_msgs::Vector3 center = msg->center;
    geometry_msgs::Vector3 normal = msg->normal;
    target_pose_.center << center.x, center.y, center.z;
    target_pose_.normal << normal.x, normal.y, normal.z;

    double theta = atan2(normal.y, normal.x);
    double yaw = theta + M_PI;
    state.pos(0) = center.x + distance * cos(theta);
    state.pos(1) = center.y + distance * sin(theta);
    state.pos(2) = center.z;
    state.yaw = yaw;
    state.only_pose = true;

    return state;
}

void TrajServer::visualizeTarget(const Eigen::Vector3d &pos)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos(0);
    marker.pose.position.y = pos(1);
    marker.pose.position.z = pos(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    target_marker_pub_.publish(marker);
}

std::pair<double, double> TrajServer::calculate_yaw(double t_cur, NavigationMode mode)
{
    // TODO: implement this
    double yaw = 0.0;
    double yaw_rate = 0.0;
    if (mode == NavigationMode::ROOT_TRACKING)
    {
        Eigen::Vector3d cmd_pos = traj_->getPos(t_cur);
        double y = target_pose_.center(1) - cmd_pos(1);
        double x = target_pose_.center(0) - cmd_pos(0);
        yaw = atan2(y, x);
        yaw_rate = 0.0;
    }
    else{
        if (t_cur < traj_duration_ && t_cur >= 0.0)
        {
            Eigen::Vector3d vel = traj_->getVel(t_cur);
            yaw = atan2(vel(1), vel(0));
            yaw_rate = 0.0;
        }
    }
    return std::make_pair(yaw, yaw_rate);
}

void TrajServer::cmdTimerCallback(const ros::TimerEvent &event)
{
    if(mode_ == NavigationMode::INACTIVE)
    {
        return;
    }

    ros::Time time_now = ros::Time::now();
    if ((time_now - heartbeat_time_).toSec() > 0.5)
    {
        ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");
    }

    if(mode_ == NavigationMode::SEARCH || mode_ == NavigationMode::APPROACH)
    {
        double t_cur = (time_now - traj_start_time_).toSec();
        if (t_cur < traj_duration_ && t_cur >= 0.0)
        {
            // PolynomialTrajectoryを追従
            DroneState state;
            state.pos = traj_->getPos(t_cur);
            state.vel = traj_->getVel(t_cur);
            state.acc = traj_->getAcc(t_cur);
            state.jerk = traj_->getJer(t_cur);
            std::pair<double, double> yaw_yawdot(0, 0);
            yaw_yawdot = calculate_yaw(t_cur, mode_);
            state.yaw = yaw_yawdot.first;
            state.yaw_rate = yaw_yawdot.second;
            publishCmd(state);
        }
        else if (use_pin_cmd_)
        {
            publishPinCmd();
        }
    }
    else if(mode_ == NavigationMode::ROOT_TRACKING)
    {
        // ROS_ERROR("Not Implemented ROOT_TRACKING!");
        double t_cur = (time_now - traj_start_time_).toSec();
        if (t_cur < traj_duration_ && t_cur >= 0.0)
        {
            // PolynomialTrajectoryを追従
            DroneState state;
            state.pos = traj_->getPos(t_cur);
            state.vel = traj_->getVel(t_cur);
            state.acc = traj_->getAcc(t_cur);
            state.jerk = traj_->getJer(t_cur);
            std::pair<double, double> yaw_yawdot(0, 0);
            yaw_yawdot = calculate_yaw(t_cur, mode_);
            state.yaw = yaw_yawdot.first;
            state.yaw_rate = yaw_yawdot.second;
            publishCmd(state);
        }
        else if (use_pin_cmd_)
        {
            publishPinCmd();
        }
    }
    else if(mode_ == NavigationMode::PURE_TRACKING)
    {
        publishCmd(tracking_state_);
    }
    else if(mode_ == NavigationMode::HOVERING)
    {
        publishHoverCmd();
    }
    else
    {
        ROS_ERROR("[traj_server] Unrecognized mode!");
    }
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