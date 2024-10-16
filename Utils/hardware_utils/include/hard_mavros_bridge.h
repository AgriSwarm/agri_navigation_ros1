#ifndef HARD_MAVROS_BRIDGE_H
#define HARD_MAVROS_BRIDGE_H

#include <fstream>
#include <string>

#include "ros/ros.h"
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>                                                                                                                                                                          

#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/StreamRate.h>

#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <swarm_msgs/SystemStatus.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_srvs/SetBool.h>

#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_utils/PIDConfig.h>
#include <boost/thread/recursive_mutex.hpp>

class MavrosBridge
{
public:
    MavrosBridge();

    struct ParamPair {
        std::string name;
        double hardware_utils::PIDConfig::*value;
    };

    static const std::vector<ParamPair> params;
private:
    void rcCallback(mavros_msgs::RCIn msg);
    // void activateCallback(std_msgs::Bool msg);
    bool activateCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    // bool takeoffCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void stateCallback(mavros_msgs::State msg);
    void pubPictgramState(mavros_msgs::State msg);
    void hpCallback(mavros_msgs::HomePosition msg);
    void batteryCallback(sensor_msgs::BatteryState msg);
    void odomCallback(nav_msgs::Odometry msg);
    void visionPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void statusCallback(swarm_msgs::SystemStatus msg);
    void poseImuCallback(const geometry_msgs::PoseStampedConstPtr& pose, const sensor_msgs::ImuConstPtr& imu);
    bool checkMove(void);
    sensor_msgs::Joy convertRCtoJoy(const mavros_msgs::RCIn& msg);
    void initialSetup(void);
    bool activate(bool activate);
    // bool takeoff(float altitude);

    void setupStreamRate();
    void thermalTimerCallback(const ros::TimerEvent& event);
    // void vioAlignTimerCallback(const ros::TimerEvent& event);
    void pictStateTimerCallback(const ros::TimerEvent&);
    void publishTemp(const std::string& file_path, ros::Publisher& publisher);

    hardware_utils::PIDConfig getPIDParam();

    std::tuple<bool, int, float> getParam(const std::string& param);
    std::tuple<bool, int, float> setParam(const std::string& param, int value_integer, float value_real=0.0f);
    void configCallback(hardware_utils::PIDConfig& config, uint32_t level);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher joy_pub_;
    ros::Publisher battery_pub_, vision_pose_pub_, status_pub_;
    ros::Subscriber rc_sub_;
    // ros::Subscriber activate_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber hp_sub_;
    ros::Subscriber battery_sub_, odom_sub_, status_sub_, vision_pose_sub_;
    ros::ServiceServer activate_srv_, takeoff_srv_;

    boost::recursive_mutex config_mutex_;
    dynamic_reconfigure::Server<hardware_utils::PIDConfig> server_;
    
    // message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    // message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Imu> SyncPolicyPoseImu;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicyPoseImu>> sync_pose_imu_;

    ros::Publisher pub_temp0_;
    ros::Publisher pub_temp1_;
    ros::Timer thermal_timer_, vio_align_timer_, pict_state_timer_;

    ros::Publisher set_gp_origin_pub_, odom_pub_, setpoint_pos_pub_, setpoint_raw_pub_, pict_state_pub_;
    ros::ServiceClient mode_client_;
    ros::ServiceClient arm_client_;
    ros::ServiceClient set_msg_rate_group_client_;
    ros::ServiceClient set_msg_rate_client_;
    ros::ServiceClient get_param_client_, set_param_client_, pull_param_client_;

    tf::TransformBroadcaster br_;
    tf::Transform transform_;

    hardware_utils::PIDConfig config_last_;

    mavros_msgs::State last_state_;
    nav_msgs::Odometry odom_cur_;
    bool hp_valid_{ false };
    bool ap_initialized_{ false };
    bool nav_initialized_{ false };
    bool set_params_{ false };
    bool ap_connected_{ false };
    bool init_mavparams_{ false };
    int cells_batt_{ 0 };
    int imu_freq_{ 0 };
    int infra_freq_{ 0 };
    int self_id{ 0 };
    float vio_align_interval_{ 1.0 };
};

#endif // HARD_MAVROS_BRIDGE_H