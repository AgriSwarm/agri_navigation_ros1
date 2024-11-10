// traj_server.cpp
#include <plan_manage/traj_server.h>

namespace ego_planner
{

void TrajServer::resetTraj()
{
    traj_.reset(new poly_traj::Trajectory());
    traj_duration_ = 0.0;
    traj_start_time_ = ros::Time(0);
    traj_id_ = -1;
    first_sensing_ = true;
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


}  // namespace ego_planner