// traj_server.cpp
#include <plan_manage/traj_server.h>

namespace ego_planner
{

bool TrajServer::updateModeCallback(quadrotor_msgs::UpdateMode::Request& req,
                        quadrotor_msgs::UpdateMode::Response& res)
{
    if (req.mode == quadrotor_msgs::UpdateMode::Request::IDLE)
    {
        updateMode(NavigationMode::IDLE);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::SEARCH)
    {
        updateMode(NavigationMode::SEARCH);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::APPROACH)
    {
        updateMode(NavigationMode::APPROACH);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::ROOT_TRACK)
    {
        updateMode(NavigationMode::ROOT_TRACK);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::PURE_TRACK)
    {
        updateMode(NavigationMode::PURE_TRACK);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::HOVER)
    {
        updateMode(NavigationMode::HOVER);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::POSHOLD)
    {
        updateMode(NavigationMode::POSHOLD);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::TURN_FOR_PERSUIT)
    {
        updateMode(NavigationMode::TURN_FOR_PERSUIT);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::TURN_FOR_ESCAPE)
    {
        updateMode(NavigationMode::TURN_FOR_ESCAPE);
    }
    else if (req.mode == quadrotor_msgs::UpdateMode::Request::ESCAPE)
    {
        updateMode(NavigationMode::ESCAPE);
    }
    else
    {
        ROS_ERROR("[traj_server] Unrecognized mode!");
    }
    res.success = true;
    return true;
}

void TrajServer::statusCallback(const swarm_msgs::SystemStatus::ConstPtr &msg)
{
    status_cur_.infra_status = msg->infra_status;
    status_cur_.ap_status = msg->ap_status;
    if (status_cur_.infra_status != swarm_msgs::SystemStatus::INFRA_ARMED ||
        status_cur_.ap_status != swarm_msgs::SystemStatus::AP_GUIDED)
    {
        updateMode(NavigationMode::IDLE);
    }
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

void TrajServer::emergencyStopCallback(const std_msgs::Empty &msg)
{
    // TODO: implement emergency stop
    updateMode(NavigationMode::POSHOLD);
    resetTraj();
}

void TrajServer::conservativePersuitCallback(const quadrotor_msgs::GoalSet::ConstPtr &msg)
{
    if (msg->drone_id != drone_id_)
    {
        return;
    }
    reserved_goal_ = Eigen::Vector3d(msg->goal[0], msg->goal[1], msg->goal[2]);
    // calculate yaw
    Eigen::Vector3d diff = reserved_goal_ - odom_state_.pos;
    
    pursuit_state_.pos << odom_state_.pos(0), odom_state_.pos(1), odom_state_.pos(2);
    double yaw = atan2(diff(1), diff(0));
    pursuit_state_.yaw = yaw;
    pursuit_state_.only_pose = true;
    updateMode(NavigationMode::TURN_FOR_PERSUIT);
}

void TrajServer::conservativeEscapeCallback(const quadrotor_msgs::GoalSet::ConstPtr &msg)
{
    if (msg->drone_id != drone_id_)
    {
        return;
    }
    updateMode(NavigationMode::ESCAPE);
    publishGoal(Eigen::Vector3d(msg->goal[0], msg->goal[1], msg->goal[2]));

    // float escape_distance = 0.5;
    // reserved_goal_ = Eigen::Vector3d(msg->goal[0], msg->goal[1], msg->goal[2]);
    // Eigen::Vector3d backword = Eigen::Vector3d(
    //     -cos(odom_state_.yaw)*escape_distance+odom_state_.pos(0), 
    //     -sin(odom_state_.yaw)*escape_distance+odom_state_.pos(1), 
    //     odom_state_.pos(2));

    // Eigen::Vector3d diff = reserved_goal_ - odom_state_.pos;
    // escape_cmd_state_ = odom_state_;
    
    // pursuit_state_.pos << backword(0), backword(1), backword(2);
    // double yaw = atan2(diff(1), diff(0));
    // pursuit_state_.yaw = yaw;
    // pursuit_state_.only_pose = true;
    // updateMode(NavigationMode::TURN_FOR_ESCAPE);
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

    if(mode_ == NavigationMode::TURN_FOR_ESCAPE){
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

    // if (mode_ == NavigationMode::IDLE){
    //     updateMode(NavigationMode::SEARCH);
    // }

    if(mode_ == NavigationMode::APPROACH){
        return;
    }
    if(mode_ == NavigationMode::ROOT_TRACK){
        return;
    }
    if(mode_ == NavigationMode::PURE_TRACK){
        return;
    }
    if(mode_ == NavigationMode::ESCAPE){
        return;
    }

    updateMode(NavigationMode::SEARCH);
}

void TrajServer::setpointPosCallback(const swarm_msgs::PositionCommand::ConstPtr &msg)
{
    if (msg->drone_id != drone_id_)
    {
        return;
    }
    DroneState state;
    state.pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    state.yaw = tf::getYaw(msg->pose.orientation);
    state.only_pose = true;
    publishCmd(state);
    updateMode(NavigationMode::POSHOLD);
}

void TrajServer::targetPoseCallback(const quadrotor_msgs::TrackingPose::ConstPtr &msg)
{
    // ROS_INFO("[traj_server] targetPoseCallback");
    if (msg->drone_id != drone_id_ || mode_ == NavigationMode::ESCAPE || mode_ == NavigationMode::TURN_FOR_ESCAPE)
    {
        // ROS_INFO("[traj_server] drone_id: %d, msg->drone_id: %d, escape_mode_: %d", drone_id_, msg->drone_id, escape_mode_);
        return;
    }
    if(msg->target_status == quadrotor_msgs::TrackingPose::TARGET_STATUS_APPROXIMATE)
    {
        // ROS_INFO("[traj_server] Mode changed from %s to APPROACH", modeToString(mode_).c_str());
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
        Eigen::Vector3d pos_diff = tracking_state_.pos - odom_state_.pos;
        double yaw_diff = fabs(tracking_state_.yaw - odom_state_.yaw);
        while(yaw_diff > 2 * M_PI)
        {
            yaw_diff -= 2 * M_PI;
        }
        // std::cout << "tracking_state_.yaw: " << tracking_state_.yaw << ", odom_state_.yaw: " << odom_state_.yaw << std::endl;
        // std::cout << "odom_state_.yaw: " << odom_state_.yaw << std::endl;

        if((pos_diff.norm() > root_tracking_threshold_) || (yaw_diff > root_tracking_yaw_threshold_))
        {
            // ROS_INFO("[traj_server] Mode changed from %s to ROOT_TRACK", modeToString(mode_).c_str());
            updateMode(NavigationMode::ROOT_TRACK);
            executeRootTracking(tracking_state_);
        }
        else
        {
            // ROS_INFO("[traj_server] Mode changed from %s to PURE_TRACK", modeToString(mode_).c_str());
            updateMode(NavigationMode::PURE_TRACK);
        }
    }
    else if(msg->target_status == quadrotor_msgs::TrackingPose::TARGET_STATUS_LOST)
    {
        // ROS_INFO("[traj_server] Mode changed from %s to SEARCH", modeToString(mode_).c_str());
        updateMode(NavigationMode::SEARCH);
        resetTraj();
    }
    else
    {
        ROS_ERROR("[traj_server] Unrecognized target status!");
        updateMode(NavigationMode::IDLE);
    }
}

void TrajServer::cmdTimerCallback(const ros::TimerEvent &event)
{
    if(mode_ == NavigationMode::IDLE)
    {
        return;
    }

    ros::Time time_now = ros::Time::now();
    if ((time_now - heartbeat_time_).toSec() > 0.5)
    {
        ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");
    }

    if(mode_ == NavigationMode::SEARCH || mode_ == NavigationMode::APPROACH || mode_ == NavigationMode::ROOT_TRACK || mode_ == NavigationMode::ESCAPE)
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
    else if(mode_ == NavigationMode::PURE_TRACK)
    {
        publishCmd(tracking_state_);
    }
    else if(mode_ == NavigationMode::HOVER)
    {
        publishHoverCmd();
    }
    else if(mode_ == NavigationMode::POSHOLD)
    {
        publishPinCmd();
    }
    else if(mode_ == NavigationMode::TURN_FOR_PERSUIT)
    {
        if(PureTargetControl(pursuit_state_))
        {
            ros::Duration(2.0).sleep();
            updateMode(NavigationMode::POSHOLD);
            publishGoal(reserved_goal_);
        }
    }
    else if(mode_ == NavigationMode::TURN_FOR_ESCAPE)
    {
        if(SequentialTargetControl(pursuit_state_))
        {
            ros::Duration(2.0).sleep();
            updateMode(NavigationMode::POSHOLD);
            publishGoal(reserved_goal_);
        }
    }
    else
    {
        ROS_ERROR("[traj_server] Unrecognized mode!");
    }
}

}  // namespace ego_planner