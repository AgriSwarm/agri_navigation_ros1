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

class MavrosBridge
{
public:
    MavrosBridge();

private:
    void rcCallback(mavros_msgs::RCIn msg);
    void activateCallback(std_msgs::Bool msg);
    void stateCallback(mavros_msgs::State msg);
    void hpCallback(mavros_msgs::HomePosition msg);
    void batteryCallback(sensor_msgs::BatteryState msg);
    void odomCallback(nav_msgs::Odometry msg);
    bool checkMove(void);
    sensor_msgs::Joy convertRCtoJoy(const mavros_msgs::RCIn& msg);
    void initialSetup(void);
    bool activate(bool activate);

    void setupMavParams();
    void thermalTimerCallback(const ros::TimerEvent& event);
    void publishTemp(const std::string& file_path, ros::Publisher& publisher);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher joy_pub_;
    ros::Publisher battery_pub_, vision_pose_pub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber activate_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber hp_sub_;
    ros::Subscriber battery_sub_, odom_sub_;

    ros::Publisher pub_temp0_;
    ros::Publisher pub_temp1_;
    ros::Timer thermal_timer_;

    ros::Publisher set_gp_origin_pub_;
    ros::ServiceClient mode_client_;
    ros::ServiceClient arm_client_;
    ros::ServiceClient set_msg_rate_group_client_;
    ros::ServiceClient set_msg_rate_client_;

    mavros_msgs::State last_state_;
    bool hp_valid_{ false };
    bool initialized_{ false };
    bool set_params_{ false };
    int cells_batt_{ 0 };
    int imu_freq_{ 0 };
};

#endif // HARD_MAVROS_BRIDGE_H