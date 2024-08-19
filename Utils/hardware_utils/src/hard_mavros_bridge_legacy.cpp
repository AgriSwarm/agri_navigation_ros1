#include "ros/ros.h"

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/StreamRate.h>

class MavrosBridge
{
public:
  MavrosBridge() : nh_(), pnh_("~")
  {
    pnh_.param<int>("cells_batt", cells_batt_, 2);
    pnh_.param<int>("imu_freq", imu_freq_, 100);

    joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/mavros_bridge/joy", 10);
    set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
    battery_pub_ = nh_.advertise<std_msgs::Float32>("/mavros_bridge/battery", 10);

    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // pull_param_client_ = nh_.serviceClient<mavros_msgs::ParamPull>("/mavros/param/pull");
    // set_param_client_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    set_msg_rate_group_client_ = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    set_msg_rate_client_ = nh_.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");

    rc_sub_ = nh_.subscribe("/mavros/rc/in", 1, &MavrosBridge::rcCallback, this);
    activate_sub_ = nh_.subscribe("/mavros_bridge/activate", 1, &MavrosBridge::activateCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 1, &MavrosBridge::stateCallback, this);
    hp_sub_ = nh_.subscribe("/mavros/home_position/home", 1, &MavrosBridge::hpCallback, this);
    battery_sub_ = nh_.subscribe("/mavros/battery", 1, &MavrosBridge::batteryCallback, this);

    setupMavParams();
  }

  void rcCallback(mavros_msgs::RCIn msg)
  {
    sensor_msgs::Joy joy = convertRCtoJoy(msg);
    joy_pub_.publish(joy);
  }

  void activateCallback(std_msgs::Bool msg)
  {
    ROS_INFO("activate %u", msg.data);
    activate(msg.data);
  }

  void stateCallback(mavros_msgs::State msg)
  {
    last_state_ = msg;

    if (!initialized_)
    {
      initialSetup();
      initialized_ = true;
    }
  }

  void hpCallback(mavros_msgs::HomePosition msg)
  {
    hp_valid_ = true;
  }

  void batteryCallback(sensor_msgs::BatteryState msg)
  {
    // 3.4 ~ 4.2 V * cells_batt_ -> 0 ~ 100 %
    float voltage = msg.voltage;
    float min_voltage = 3.4 * cells_batt_;
    float max_voltage = 4.2 * cells_batt_;
    if (voltage < min_voltage){
      voltage = min_voltage;
      // ROS_WARN("Battery voltage is too low: %f", voltage);
    }
    if (voltage > max_voltage){
      voltage = max_voltage;
      // ROS_WARN("Battery voltage is too high: %f", voltage);
    }
    int battery = (voltage - min_voltage) / (max_voltage - min_voltage) * 100;
    std_msgs::Float32 battery_msg;
    battery_msg.data = battery;
    battery_pub_.publish(battery_msg);
  }

  bool checkMove(void)
  {
    bool guided = last_state_.mode == "GUIDED";
    return last_state_.armed && guided && hp_valid_;
  }

  sensor_msgs::Joy convertRCtoJoy(const mavros_msgs::RCIn& msg)
  {
    sensor_msgs::Joy joy;
    joy.axes.resize(4);
    joy.buttons.resize(1);
    if (5 <= msg.channels.size())
    {
      float x = -((float)msg.channels[2] - 1510) / 410;
      float y = -((float)msg.channels[0] - 1510) / 410;
      float z = -((float)msg.channels[1] - 1510) / 410;
      float r = -((float)msg.channels[3] - 1510) / 410;
      joy.axes[0] = x;
      joy.axes[1] = y;
      joy.axes[2] = z;
      joy.axes[3] = r;

      auto getButton = [](const int b) {
        int output = 0;
        if (b < 1300)
          output = -1;
        else if (b < 1700)
          output = 0;
        else
          output = 1;
        return output;
      };
      joy.buttons[0] = getButton(msg.channels[4]);
    }
    return joy;
  }

  void initialSetup(void)
  {
    ROS_INFO("set GP origin");
    geographic_msgs::GeoPointStamped geo;
    set_gp_origin_pub_.publish(geo);
  }

  bool activate(bool activate)
  {
    ROS_INFO("set GUIDED");
    mavros_msgs::SetMode mode;
    mode.request.base_mode = 0;
    mode.request.custom_mode = "GUIDED";
    mode_client_.call(mode);

    ROS_INFO("set %s", activate ? "arm" : "disarm");
    mavros_msgs::CommandBool cmd;
    cmd.request.value = activate;
    arm_client_.call(cmd);

    return true;
  }

private:
  void setupMavParams()
  {
    ros::NodeHandle nh;
    ros::Rate rate(1); // 1 Hz rate for checking service availability

    // // Wait for the pull_param service to become available
    // while (ros::ok() && !set_msg_rate_group_client_.waitForExistence(ros::Duration(1.0))) {
    //   // ROS_INFO("Waiting for /mavros/set_stream_rate service to become available...");
    //   rate.sleep();
    // }

    // ros::Duration(3.0).sleep();

    // ROS_INFO("Disable all streams");
    // mavros_msgs::StreamRate stream_rate_srv;
    // stream_rate_srv.request.stream_id = 0;
    // stream_rate_srv.request.message_rate = 1;
    // stream_rate_srv.request.on_off = false;
    // if (!set_msg_rate_group_client_.call(stream_rate_srv)) {
    //   ROS_ERROR("Failed to call service /mavros/set_stream_rate");
    //   return;
    // }

    // Wait for the set_param service to become available
    while (ros::ok() && !set_msg_rate_client_.waitForExistence(ros::Duration(1.0))) {
      // ROS_INFO("Waiting for /mavros/set_message_interval service to become available...");
      rate.sleep();
    }

    ros::Duration(3.0).sleep();

    ROS_INFO("Set IMU message rate to %d Hz", imu_freq_);
    mavros_msgs::MessageInterval msg_interval_srv;
    msg_interval_srv.request.message_id = 27;
    msg_interval_srv.request.message_rate = imu_freq_;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
      ROS_ERROR("Failed to call service /mavros/set_message_interval");
      return;
    }
    ROS_INFO("Set Battery message rate to %d Hz", 5);
    msg_interval_srv.request.message_id = 147;
    msg_interval_srv.request.message_rate = 5;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
      ROS_ERROR("Failed to call service /mavros/set_message_interval");
      return;
    }

    ROS_INFO("Stream rate setup complete !");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher joy_pub_;
  ros::Publisher battery_pub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber activate_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber hp_sub_;
  ros::Subscriber battery_sub_;

  // for initial setup
  ros::Publisher set_gp_origin_pub_;
  ros::ServiceClient mode_client_;
  ros::ServiceClient arm_client_;
  // ros::ServiceClient pull_param_client_;
  // ros::ServiceClient set_param_client_;
  ros::ServiceClient set_msg_rate_group_client_;
  ros::ServiceClient set_msg_rate_client_;

  mavros_msgs::State last_state_;
  bool hp_valid_{ false };
  bool initialized_{ false };
  int cells_batt_{ 0 };
  int imu_freq_{ 0 };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavros_bridge");
  MavrosBridge mavros_bridge;
  ros::spin();
  return 0;
}
