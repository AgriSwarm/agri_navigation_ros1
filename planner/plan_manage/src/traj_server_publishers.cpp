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
        ROS_ERROR("[traj_server] Not ready to receive setpoint position command!");
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

void TrajServer::publishCmd(const DroneState &state)
{
    if(state.pos.norm() < 1e-6 && state.vel.norm() < 1e-6 && state.acc.norm() < 1e-6)
    {
        ROS_ERROR("[traj_server] Invalid state command!");
        return;
    }
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

}  // namespace ego_planner