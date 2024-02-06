#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/TrackingPose.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/FullState.h>
#include <quadrotor_msgs/Position.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;
ros::Publisher setpoint_mavros_pub;
ros::Publisher cmd_full_state_pub;
ros::Publisher cmd_position_pub;

quadrotor_msgs::PositionCommand legacy_cmd;
mavros_msgs::PositionTarget mavros_cmd;
quadrotor_msgs::FullState full_state_cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
bool activated_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
ros::Time last_cmd_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
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

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
  activated_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void publish_cf_cmd(
  Vector3d &p, 
  Vector3d &v, 
  Vector3d &a, 
  Vector3d &j, 
  double y, 
  double yd, 
  std_msgs::Header &header)
{
  full_state_cmd.header = header;
  full_state_cmd.pose.position.x = p(0);
  full_state_cmd.pose.position.y = p(1);
  full_state_cmd.pose.position.z = p(2);
  Quaterniond q(Eigen::AngleAxisd(y, Vector3d::UnitZ()));
  full_state_cmd.pose.orientation.w = q.w();
  full_state_cmd.pose.orientation.x = q.x();
  full_state_cmd.pose.orientation.y = q.y();
  full_state_cmd.pose.orientation.z = q.z();
  full_state_cmd.twist.linear.x = v(0);
  full_state_cmd.twist.linear.y = v(1);
  full_state_cmd.twist.linear.z = v(2);
  // full_state_cmd.twist.angular.z = yd;
  full_state_cmd.acc.x = a(0);
  full_state_cmd.acc.y = a(1);
  full_state_cmd.acc.z = a(2);
  cmd_full_state_pub.publish(full_state_cmd);
}

void publish_mavros_cmd(
  Vector3d &p, 
  Vector3d &v, 
  Vector3d &a, 
  Vector3d &j, 
  double y, 
  double yd, 
  std_msgs::Header &header)
{
  mavros_cmd.header = header;
  mavros_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  mavros_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  mavros_cmd.position.x = p(0);
  mavros_cmd.position.y = p(1);
  mavros_cmd.position.z = p(2);
  mavros_cmd.velocity.x = v(0);
  mavros_cmd.velocity.y = v(1);
  mavros_cmd.velocity.z = v(2);
  mavros_cmd.acceleration_or_force.x = a(0);
  mavros_cmd.acceleration_or_force.y = a(1);
  mavros_cmd.acceleration_or_force.z = a(2);
  mavros_cmd.yaw = y;
  mavros_cmd.yaw_rate = yd;
  setpoint_mavros_pub.publish(mavros_cmd);
}

void publish_legacy_cmd(
  Vector3d &p, 
  Vector3d &v, 
  Vector3d &a, 
  Vector3d &j, 
  double y, 
  double yd, 
  std_msgs::Header &header)
{
  legacy_cmd.header = header;
  legacy_cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  legacy_cmd.trajectory_id = traj_id_;
  legacy_cmd.position.x = p(0);
  legacy_cmd.position.y = p(1);
  legacy_cmd.position.z = p(2);
  legacy_cmd.velocity.x = v(0);
  legacy_cmd.velocity.y = v(1);
  legacy_cmd.velocity.z = v(2);
  legacy_cmd.acceleration.x = a(0);
  legacy_cmd.acceleration.y = a(1);
  legacy_cmd.acceleration.z = a(2);
  legacy_cmd.jerk.x = j(0);
  legacy_cmd.jerk.y = j(1);
  legacy_cmd.jerk.z = j(2);
  legacy_cmd.yaw = y;
  legacy_cmd.yaw_dot = yd;
  pos_cmd_pub.publish(legacy_cmd);
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";
  publish_legacy_cmd(p, v, a, j, y, yd, header);
  publish_mavros_cmd(p, v, a, j, y, yd, header);
  publish_cf_cmd(p, v, a, j, y, yd, header);
  last_pos_ = p;
}

void trackingPoseCallback(traj_utils::TrackingPose msg)
{
  float distance = msg.distance;
  geometry_msgs::Vector3 centor = msg.center;
  geometry_msgs::Vector3 normal = msg.normal;
  quadrotor_msgs::Position cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";

  double theta = atan2(normal.y, normal.x);
  double yaw = theta + M_PI;
  cmd.x = centor.x + distance * cos(theta);
  cmd.y = centor.y + distance * sin(theta);
  cmd.z = centor.z;
  cmd.yaw = yaw;

  Vector3d pos(cmd.x, cmd.y, cmd.z);
  Vector3d vel(Vector3d::Zero());
  Vector3d acc(Vector3d::Zero());
  Vector3d jer(Vector3d::Zero());
  // publish_cmd(pos, vel, acc, jer, yaw, 0);
  cmd_position_pub.publish(cmd);
  publish_legacy_cmd(
    pos, 
    vel, 
    acc, 
    jer, 
    yaw,
    0, 
    cmd.header);
  last_pos_ = pos;
  last_yaw_ = yaw;
}


void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
    last_cmd_time_ = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }
  else if (t_cur >= traj_duration_ && activated_)
  {
    // hover when finished traj_
    // pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "world";
    publish_cf_cmd(last_pos_, vel, acc, jer, last_yaw_, 0, header);
    if (time_now - last_cmd_time_ > ros::Duration(10.0))
    {
      activated_ = false;
    }

#if FLIP_YAW_AT_END
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#endif
  }

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber tracking_pose_sub = nh.subscribe("planning/track_pose", 10, trackingPoseCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  setpoint_mavros_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);
  cmd_full_state_pub = nh.advertise<quadrotor_msgs::FullState>("/cmd_full_state", 50);
  cmd_position_pub = nh.advertise<quadrotor_msgs::Position>("/cmd_position", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}