#include "hard_mavros_bridge.h"

MavrosBridge::MavrosBridge() : nh_(), pnh_("~"), server_(config_mutex_) 
{
    pnh_.param<int>("cells_batt", cells_batt_, 2);
    pnh_.param<int>("imu_freq", imu_freq_, 100);
    pnh_.param<int>("infra_freq", infra_freq_, 5);
    pnh_.param<int>("drone_id", self_id, 0);
    pnh_.param<bool>("set_params", set_params_, false);
    pnh_.param<float>("vio_align_interval", vio_align_interval_, 1.0);
    pnh_.param<std::string>("ap_param_type", ap_param_type_, "vanilla");

    joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/mavros_bridge/joy", 10);
    set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
    battery_pub_ = nh_.advertise<std_msgs::Float32>("/mavros_bridge/battery", 10);
    // vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    status_pub_ = nh_.advertise<swarm_msgs::SystemStatus>("/hardware_bridge/status", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/mavros_bridge/ap_odom", 10);
    setpoint_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    pict_state_pub_ = nh_.advertise<jsk_rviz_plugins::Pictogram>("/mavros_bridge/pictogram", 10);

    pub_temp0_ = nh_.advertise<std_msgs::Float32>("/hardware_bridge/cpu_temperature", 1);
    pub_temp1_ = nh_.advertise<std_msgs::Float32>("/hardware_bridge/gpu_temperature", 1);

    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    get_param_client_ = nh_.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    set_param_client_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    set_msg_rate_group_client_ = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    set_msg_rate_client_ = nh_.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");
    pull_param_client_ = nh_.serviceClient<mavros_msgs::ParamPull>("/mavros/param/pull");

    rc_sub_ = nh_.subscribe("/mavros/rc/in", 1, &MavrosBridge::rcCallback, this);
    // activate_sub_ = nh_.subscribe("/mavros_bridge/activate", 1, &MavrosBridge::activateCallback, this);
    activate_srv_ = nh_.advertiseService("/mavros_bridge/activate", &MavrosBridge::activateCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 1, &MavrosBridge::stateCallback, this);
    // hp_sub_ = nh_.subscribe("/mavros/home_position/home", 1, &MavrosBridge::hpCallback, this);
    battery_sub_ = nh_.subscribe("/mavros/battery", 1, &MavrosBridge::batteryCallback, this);
    // odom_sub_ = nh_.subscribe("/mavros_bridge/odom", 1, &MavrosBridge::odomCallback, this);
    vision_pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose", 1, &MavrosBridge::visionPoseCallback, this);
    
    status_sub_ = nh_.subscribe("/hardware_bridge/status", 1, &MavrosBridge::statusCallback, this);
    // takeoff_srv_ = nh_.advertiseService("/mavros_bridge/takeoff", &MavrosBridge::takeoffCallback, this);

    takeoff_mand_sub_ = nh_.subscribe("/hardware_bridge/takeoff_mand", 1, &MavrosBridge::takeoffMandCallback, this);
    land_mand_sub_ = nh_.subscribe("/hardware_bridge/land_mand", 1, &MavrosBridge::landMandCallback, this);
    takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/mavros/local_position/pose", 1));
    imu_sub_.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "/mavros/imu/data", 1));
    sync_pose_imu_.reset(new message_filters::Synchronizer<SyncPolicyPoseImu>(SyncPolicyPoseImu(10), *pose_sub_, *imu_sub_));
    sync_pose_imu_->registerCallback(boost::bind(&MavrosBridge::poseImuCallback, this, _1, _2));

    thermal_timer_ = nh_.createTimer(ros::Duration(1.0), &MavrosBridge::thermalTimerCallback, this);
    pict_state_timer_ = nh_.createTimer(ros::Duration(1.0), &MavrosBridge::pictStateTimerCallback, this);

    if (!initGPIO()) {
        ROS_ERROR("Failed to initialize GPIO");
        return;
    }
    
    rotate_motor_srv_ = nh_.advertiseService("rotate_motor", 
            &MavrosBridge::rotateMotorCallback, this);

    if (set_params_){
        setupStreamRate();
        // pullAndSetParams();
        hardware_utils::PIDConfig initial_config = getPIDParam();
        server_.setConfigDefault(initial_config);
        server_.updateConfig(initial_config);
        
        config_last_ = initial_config;

        dynamic_reconfigure::Server<hardware_utils::PIDConfig>::CallbackType f;
        f = boost::bind(&MavrosBridge::configCallback, this, _1, _2);
        server_.setCallback(f);
    }
}

void MavrosBridge::initialSetup(void)
{
    ROS_INFO("set GP origin");
    geographic_msgs::GeoPointStamped geo;
    set_gp_origin_pub_.publish(geo);
}

bool MavrosBridge::activate(bool activate)
{
    if(!ap_initialized_)
    {
        ROS_ERROR("AP not initialized");
        return false;
    }
    if(!nav_initialized_)
    {
        ROS_ERROR("Nav not initialized");
        return false;
    }
    
    ROS_INFO("set GUIDED");
    mavros_msgs::SetMode mode;
    mode.request.base_mode = 0;
    mode.request.custom_mode = "GUIDED";
    if (!mode_client_.call(mode))
    {
        ROS_ERROR("Failed to set GUIDED");
        return false;
    }

    ROS_INFO("set %s", activate ? "arm" : "disarm");
    mavros_msgs::CommandBool cmd;
    cmd.request.value = activate;
    if (!arm_client_.call(cmd))
    {
        ROS_ERROR("Failed to %s", activate ? "arm" : "disarm");
        return false;
    }

    if (activate)
    {
        swarm_msgs::SystemStatus status;
        status.status_id = swarm_msgs::SystemStatus::ACTIVATE;
        status.drone_id = self_id;
        status_pub_.publish(status);
    }
    return true;
}

MavrosBridge::~MavrosBridge()
{
    cleanupGPIO();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_bridge");
    MavrosBridge mavros_bridge;
    ros::spin();
    return 0;
}