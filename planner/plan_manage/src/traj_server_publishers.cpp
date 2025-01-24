// traj_server.cpp
#include <plan_manage/traj_server.h>

namespace ego_planner
{

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

void TrajServer::publishMavrosCmd(const DroneState &state)
{
    if (status_cur_.infra_status != swarm_msgs::SystemStatus::INFRA_ARMED ||
        status_cur_.ap_status != swarm_msgs::SystemStatus::AP_GUIDED)
    {
        ROS_ERROR("[TrajServer::publishMavrosCmd] Not ready to receive setpoint position command!");
        return;
    }
    
    if(state.only_pose)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = state.pos(0);
        pose.pose.position.y = state.pos(1);
        pose.pose.position.z = state.pos(2);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw);
        setpoint_pos_pub.publish(pose);
    }else{
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
        setpoint_raw_pub.publish(cmd);

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
}

bool TrajServer::PureTargetControl(const DroneState &target_state)
{
    if (!odom_received_)
    {
        ROS_ERROR("[TrajServer::PureTargetControl] Not found odom state");
        return false;
    }

    // -----------------------
    // 1. 誤差計算
    // -----------------------
    Eigen::Vector3d pos_error = target_state.pos - odom_state_.pos;
    double yaw_error = target_state.yaw - odom_state_.yaw;
    // yaw 誤差は -pi ~ pi の範囲に合わせる
    while (yaw_error >  M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    // -----------------------
    // 2. 一度に目標を指令せず，一定ステップ分だけ近づける
    //    （例: pos_step=0.1, yaw_step=0.05など）
    // -----------------------
    const double pos_step = 0.1;   // 1回の制御周期で近づく位置ステップ
    const double yaw_step = 0.3;  // 1回の制御周期で近づくヨーステップ

    // 次に指令する位置
    Eigen::Vector3d next_pos;
    // 誤差が pos_step より小さいときは目標へ直接指令
    // 大きいときは pos_step だけ近づく
    double dist_to_target = pos_error.norm();
    if (dist_to_target < pos_step)
    {
        next_pos = target_state.pos; 
    }
    else
    {
        next_pos = odom_state_.pos + pos_step * pos_error.normalized();
    }

    // 次に指令するyaw
    double next_yaw;
    double abs_yaw_error = std::fabs(yaw_error);
    if (abs_yaw_error < yaw_step)
    {
        next_yaw = target_state.yaw;
    }
    else
    {
        // 符号を考慮して yaw_step だけ近づく
        double sign = (yaw_error > 0.0) ? 1.0 : -1.0;
        next_yaw = odom_state_.yaw + sign * yaw_step;
        // 範囲 [-pi, pi] に正規化
        if (next_yaw >  M_PI) next_yaw -= 2.0 * M_PI;
        if (next_yaw < -M_PI) next_yaw += 2.0 * M_PI;
    }

    // -----------------------
    // 3. コマンド生成
    // -----------------------
    DroneState cmd_state;
    cmd_state.pos      = next_pos;
    cmd_state.vel      = Eigen::Vector3d::Zero(); // 今回はステップで姿勢制御するので速度は0に
    cmd_state.acc      = Eigen::Vector3d::Zero();
    cmd_state.jerk     = Eigen::Vector3d::Zero();
    cmd_state.snap     = Eigen::Vector3d::Zero();
    cmd_state.yaw      = next_yaw;
    cmd_state.yaw_rate = 0.0;
    cmd_state.yaw_acc  = 0.0;
    cmd_state.only_pose = true;

    // コマンド送信
    publishCmd(cmd_state);
    // ROS_INFO("[TrajServer::PureTargetControl] Send command: pos=(%f, %f, %f), yaw=%f", 
    //          cmd_state.pos(0), cmd_state.pos(1), cmd_state.pos(2), cmd_state.yaw);

    // -----------------------
    // 4. 目標到達判定
    // -----------------------
    // next_pos で指令した後の残り誤差
    Eigen::Vector3d final_pos_error = target_state.pos - next_pos;
    double          final_yaw_error = target_state.yaw - next_yaw;

    // yaw の誤差再正規化
    while (final_yaw_error >  M_PI) final_yaw_error -= 2.0 * M_PI;
    while (final_yaw_error < -M_PI) final_yaw_error += 2.0 * M_PI;

    bool reached = (final_pos_error.norm() < ctrl_pos_threshold_) 
                && (std::fabs(final_yaw_error) < ctrl_yaw_threshold_);

    if (reached)
    {
        ROS_INFO("[TrajServer::PureTargetControl] Target reached!");
        updateMode(NavigationMode::POSHOLD);
    }

    return reached;
}


// bool TrajServer::PureTargetControl(const DroneState &target_state)
// {
//     if (!odom_received_)
//     {
//         ROS_ERROR("[TrajServer::PureTargetControl] Not found odom state");
//         return false;
//     }

//     Eigen::Vector3d pos_error = target_state.pos - odom_state_.pos;
//     double yaw_error = target_state.yaw - odom_state_.yaw;
//     while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
//     while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
//     const double kp_pos = 1.0;
//     const double kp_yaw = 2.0;
//     double vel_limit = 2.0;
//     double yaw_rate_limit = 1.0;

//     DroneState cmd_state;
//     cmd_state.pos = target_state.pos;
//     cmd_state.vel = kp_pos * pos_error;

//     if (cmd_state.vel.norm() > vel_limit)
//     {
//         cmd_state.vel = cmd_state.vel.normalized() * vel_limit;
//     }
//     cmd_state.yaw = target_state.yaw;
//     cmd_state.yaw_rate = kp_yaw * yaw_error;
//     cmd_state.yaw_rate = std::max(std::min(cmd_state.yaw_rate, yaw_rate_limit), -yaw_rate_limit);

//     cmd_state.acc.setZero();
//     cmd_state.jerk.setZero();
//     cmd_state.snap.setZero();
//     cmd_state.yaw_acc = 0.0;
//     cmd_state.only_pose = true;

//     publishCmd(cmd_state);

//     // for visualization
//     // geometry_msgs::TwistStamped twist;
//     // twist.header.stamp = ros::Time::now();
//     // twist.header.frame_id = "base_link";
//     // twist.twist.angular.x = 0;
//     // twist.twist.angular.y = 0;
//     // twist.twist.angular.z = cmd_state.yaw_rate;
//     // twist.twist.linear.x = cmd_state.vel(0);
//     // twist.twist.linear.y = cmd_state.vel(1);
//     // twist.twist.linear.z = cmd_state.vel(2);
//     // twist_pub.publish(twist);
    
//     bool reached = (pos_error.norm() < ctrl_pos_threshold_) && (std::abs(yaw_error) < ctrl_yaw_threshold_);

//     if(reached)
//     {
//         ROS_INFO("[TrajServer::PureTargetControl] Target reached!");
//         updateMode(NavigationMode::POSHOLD);
//     }
    
//     return reached;
// }

void TrajServer::publishCmd(const DroneState &state)
{
    if(state.pos.norm() < 1e-6 && state.vel.norm() < 1e-6 && state.acc.norm() < 1e-6)
    {
        ROS_ERROR("[TrajServer::publishCmd] Invalid state command!");
        return;
    }
    publishMavrosCmd(state);
    publishFakeCmd(state);
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
        ROS_ERROR("[TrajServer::publishPinCmd] Not found last state");
    }
}

void TrajServer::publishHoverCmd()
{
    if(!odom_received_)
    {
        ROS_ERROR("[TrajServer::publishHoverCmd] Not found odom state");
        return;
    }
    DroneState state;
    state.pos = odom_state_.pos;      
    state.yaw = odom_state_.yaw;
    state.only_pose = true;
    publishCmd(state);
}

}  // namespace ego_planner