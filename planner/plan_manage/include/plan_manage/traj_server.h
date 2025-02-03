#ifndef _TRAJ_SERVER_H_
#define _TRAJ_SERVER_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <quadrotor_msgs/FullState.h>
#include <quadrotor_msgs/Position.h>
#include <quadrotor_msgs/TrackingPose.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/UpdateMode.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <swarm_msgs/PositionCommand.h>
#include <swarm_msgs/SystemStatus.h>

namespace ego_planner
{

enum class NavigationMode
{ 
    IDLE = 0,
    SEARCH = 1,
    APPROACH = 2,
    ROOT_TRACK = 3,
    PURE_TRACK = 4,
    HOVER = 5,
    POSHOLD = 6,
    TURN_FOR_PERSUIT = 7,
    TURN_FOR_ESCAPE = 8,
    ESCAPE = 9
};

struct DroneState
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d jerk;
    Eigen::Vector3d snap;
    double yaw;
    double yaw_rate;
    double yaw_acc;
    bool only_pose;
    bool initialized;
};

struct NormalPose
{
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
};

class TrajServer
{
    public:
        TrajServer(ros::NodeHandle &nh);
        ~TrajServer(){};

    private:
        ros::NodeHandle nh_;
        ros::Subscriber poly_traj_sub_, target_pose_sub_, heartbeat_sub_, odom_sub_, setpoint_pos_sub_, status_sub_, conservative_persuit_sub_, emergency_stop_sub_, conservative_escape_sub_;
        // fake drone
        ros::Publisher fake_pos_cmd_pub_, status_pub_;
        // crazyflie
        ros::Publisher cf_full_state_cmd_pub_, cf_position_cmd_pub_, goal_pub_,target_marker_pub_, setpoint_raw_pub, setpoint_pos_pub, twist_pub;
        ros::ServiceServer update_mode_srv_;
        ros::Timer cmd_timer_;
        ros::Timer status_timer_;
        DroneState tracking_state_, last_cmd_state_, odom_state_, escape_cmd_state_;
        boost::shared_ptr<poly_traj::Trajectory> traj_;
        double traj_duration_;
        double update_goal_threshold_, root_tracking_threshold_, root_tracking_yaw_threshold_, update_tracking_goal_threshold_;
        double time_forward_;
        double ctrl_pos_threshold_, ctrl_yaw_threshold_;
        double last_yaw_, last_yaw_dot_;
        ros::Time traj_start_time_, heartbeat_time_;
        NavigationMode mode_;
        int traj_id_;
        int drone_id_;
        Eigen::Vector3d last_goal_pos_, reserved_goal_;
        bool first_sensing_, first_root_tracking_, use_pin_cmd_, odom_received_, cmd_received_, escape_mode_;
        DroneState last_tracking_goal_;
        NormalPose target_pose_;
        swarm_msgs::SystemStatus status_cur_;
        DroneState pursuit_state_;

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void polyTrajCallback(const traj_utils::PolyTraj::ConstPtr &msg);
        void heartbeatCallback(const std_msgs::Empty::ConstPtr &msg);
        void targetPoseCallback(const quadrotor_msgs::TrackingPose::ConstPtr &msg);
        void setpointPosCallback(const swarm_msgs::PositionCommand::ConstPtr &msg);
        bool updateModeCallback(quadrotor_msgs::UpdateMode::Request& req,
                        quadrotor_msgs::UpdateMode::Response& res);
        void conservativePersuitCallback(const quadrotor_msgs::GoalSet::ConstPtr &msg);
        void conservativeEscapeCallback(const quadrotor_msgs::GoalSet::ConstPtr &msg);

        void cmdTimerCallback(const ros::TimerEvent &event);
        void statusTimerCallback(const ros::TimerEvent &event);
        void statusCallback(const swarm_msgs::SystemStatus::ConstPtr &msg);
        void emergencyStopCallback(const std_msgs::Empty &msg);

        void publishCmd(const DroneState &state);
        void publishFakeCmd(const DroneState &state);
        // void publishCFCmd(const DroneState &state);
        // void publishCFPositionCmd(const DroneState &state);
        void publishMavrosCmd(const DroneState &state);
        void publishHoverCmd();
        void publishPinCmd();
        void publishGoal(const Eigen::Vector3d &goal_pos);
        void resetTraj();
        void updateMode(NavigationMode mode);
        void executeRootTracking(DroneState target_state);
        void visualizeTarget(const Eigen::Vector3d &pos);
        std::pair<double, double> calculate_yaw(double t_cur, NavigationMode mode);
        DroneState computeTrackingState(const quadrotor_msgs::TrackingPose::ConstPtr &msg);
        bool PureTargetControl(const DroneState &state);
        bool SequentialTargetControl(const DroneState &target_state);

        std::string modeToString(NavigationMode mode){
            switch (mode)
            {
            case NavigationMode::IDLE:
                return "IDLE";
            case NavigationMode::SEARCH:
                return "SEARCH";
            case NavigationMode::APPROACH:
                return "APPROACH";
            case NavigationMode::ROOT_TRACK:
                return "ROOT_TRACK";
            case NavigationMode::PURE_TRACK:
                return "PURE_TRACK";
            case NavigationMode::HOVER:
                return "HOVER";
            case NavigationMode::POSHOLD:
                return "POSHOLD";
            case NavigationMode::TURN_FOR_PERSUIT:
                return "TURN_FOR_PERSUIT";
            case NavigationMode::TURN_FOR_ESCAPE:
                return "TURN_FOR_ESCAPE";
            case NavigationMode::ESCAPE:
                return "ESCAPE";
            default:
                return "UNKNOWN";
            }
        
        }
};
} // namespace ego_planner

#endif // _TRAJ_SERVER_H_