#include "hard_mavros_bridge.h"

const std::vector<MavrosBridge::ParamPair> MavrosBridge::params = {
    {"ATC_ANG_RLL_P", &hardware_utils::PIDConfig::ATC_ANG_RLL_P},
    {"ATC_ANG_PIT_P", &hardware_utils::PIDConfig::ATC_ANG_PIT_P},
    {"ATC_ANG_YAW_P", &hardware_utils::PIDConfig::ATC_ANG_YAW_P},
    {"ATC_RAT_RLL_P", &hardware_utils::PIDConfig::ATC_RAT_RLL_P},
    {"ATC_RAT_RLL_I", &hardware_utils::PIDConfig::ATC_RAT_RLL_I},
    {"ATC_RAT_RLL_D", &hardware_utils::PIDConfig::ATC_RAT_RLL_D},
    {"ATC_RAT_PIT_P", &hardware_utils::PIDConfig::ATC_RAT_PIT_P},
    {"ATC_RAT_PIT_I", &hardware_utils::PIDConfig::ATC_RAT_PIT_I},
    {"ATC_RAT_PIT_D", &hardware_utils::PIDConfig::ATC_RAT_PIT_D},
    {"ATC_RAT_YAW_P", &hardware_utils::PIDConfig::ATC_RAT_YAW_P},
    {"ATC_RAT_YAW_I", &hardware_utils::PIDConfig::ATC_RAT_YAW_I},
    {"ATC_RAT_YAW_D", &hardware_utils::PIDConfig::ATC_RAT_YAW_D},
    {"PSC_POSXY_P", &hardware_utils::PIDConfig::PSC_POSXY_P},
    {"PSC_POSZ_P", &hardware_utils::PIDConfig::PSC_POSZ_P},
    {"PSC_VELXY_P", &hardware_utils::PIDConfig::PSC_VELXY_P},
    {"PSC_VELXY_I", &hardware_utils::PIDConfig::PSC_VELXY_I},
    {"PSC_VELZ_P", &hardware_utils::PIDConfig::PSC_VELZ_P},
    {"PSC_VELZ_I", &hardware_utils::PIDConfig::PSC_VELZ_I},
    {"PSC_ACCZ_P", &hardware_utils::PIDConfig::PSC_ACCZ_P},
    {"PSC_ACCZ_I", &hardware_utils::PIDConfig::PSC_ACCZ_I},
    {"PSC_ACCZ_D", &hardware_utils::PIDConfig::PSC_ACCZ_D},
    {"VISO_DELAY_MS", &hardware_utils::PIDConfig::VISO_DELAY_MS}
};

MavrosBridge::MavrosBridge() : nh_(), pnh_("~"), server_(config_mutex_) 
{
    pnh_.param<int>("cells_batt", cells_batt_, 2);
    pnh_.param<int>("imu_freq", imu_freq_, 100);
    pnh_.param<int>("infra_freq", infra_freq_, 5);
    pnh_.param<int>("drone_id", self_id, 0);
    pnh_.param<bool>("set_params", set_params_, false);
    pnh_.param<float>("vio_align_interval", vio_align_interval_, 1.0);

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

    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/mavros/local_position/pose", 1));
    imu_sub_.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "/mavros/imu/data", 1));
    sync_pose_imu_.reset(new message_filters::Synchronizer<SyncPolicyPoseImu>(SyncPolicyPoseImu(10), *pose_sub_, *imu_sub_));
    sync_pose_imu_->registerCallback(boost::bind(&MavrosBridge::poseImuCallback, this, _1, _2));

    thermal_timer_ = nh_.createTimer(ros::Duration(1.0), &MavrosBridge::thermalTimerCallback, this);
    pict_state_timer_ = nh_.createTimer(ros::Duration(1.0), &MavrosBridge::pictStateTimerCallback, this);
    // vio_align_timer_ = nh_.createTimer(ros::Duration(vio_align_interval_), &MavrosBridge::vioAlignTimerCallback, this);

    if (!initGPIO()) {
        ROS_ERROR("Failed to initialize GPIO");
        return;
    }
    
    rotate_motor_srv_ = nh_.advertiseService("rotate_motor", 
            &MavrosBridge::rotateMotorCallback, this);
    // rotate_motor_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, 
    //                                    std_srvs::SetBool::Response>
    //                                   ("rotate_motor",
    //                                    boost::bind(&MavrosBridge::rotateMotorCallback, 
    //                                              this, _1, _2));

    if (set_params_){
        setupStreamRate();
        hardware_utils::PIDConfig initial_config = getPIDParam();
        server_.setConfigDefault(initial_config);
        server_.updateConfig(initial_config);
        
        config_last_ = initial_config;

        dynamic_reconfigure::Server<hardware_utils::PIDConfig>::CallbackType f;
        f = boost::bind(&MavrosBridge::configCallback, this, _1, _2);
        server_.setCallback(f);
    }
}

void MavrosBridge::configCallback(hardware_utils::PIDConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure Request:");

    for (const auto& param : params) {
        if (config.*(param.value) != config_last_.*(param.value)) {
            setParam(param.name, 0, config.*(param.value));
            // ROS_INFO("Set %s to %f", param.name.c_str(), config.*(param.value));
        }
    }

    config_last_ = config;
}

hardware_utils::PIDConfig MavrosBridge::getPIDParam() {
    if (!init_mavparams_){
        ROS_INFO("Pulling params...");
        mavros_msgs::ParamPull srv;
        if (pull_param_client_.call(srv)) {
            if (srv.response.success)
            {
                ROS_INFO("Successfully pulled params");
            }
            else
            {
                ROS_ERROR("Failed to pull params");
            }
        } else {
            ROS_ERROR("Failed to call service ParamPull");
        }
        init_mavparams_ = true;
    }
    hardware_utils::PIDConfig config;
    std::tuple<bool, int, float> param;
    ros::Duration(2.0).sleep();
    // param = getParam("ATC_ANG_RLL_P");
    // config.ATC_ANG_RLL_P = std::get<1>(param);
    for (const auto& param : params) {
        auto result = getParam(param.name);
        bool success = std::get<0>(result);
        float value = std::get<2>(result);
        if (success) {
            config.*(param.value) = value;
        }
    }

    return config;
}

std::tuple<bool, int, float> MavrosBridge::getParam(const std::string& param) {
    mavros_msgs::ParamGet srv;
    srv.request.param_id = param;

    if (get_param_client_.call(srv)) {
        ROS_INFO("Successfully get %s to int: %ld, real: %f", param.c_str(), srv.response.value.integer, srv.response.value.real);
        return std::make_tuple(srv.response.success, 
                                srv.response.value.integer, 
                                srv.response.value.real);
    } else {
        ROS_ERROR("Failed to call service ParamGet");
        return std::make_tuple(false, 0, 0.0f);
    }
}

std::tuple<bool, int, float> MavrosBridge::setParam(const std::string& param, int value_integer, float value_real) {
    mavros_msgs::ParamSet srv;
    srv.request.param_id = param;
    srv.request.value.integer = value_integer;
    srv.request.value.real = value_real;

    if (set_param_client_.call(srv)) {
        if (srv.response.success)
        {
            ROS_INFO("Successfully set %s to int: %ld, real: %f", param.c_str(), srv.response.value.integer, srv.response.value.real);
        }
        else
        {
            ROS_ERROR("Failed to set %s", param.c_str());
        }
        return std::make_tuple(srv.response.success, 
                                srv.response.value.integer, 
                                srv.response.value.real);
    } else {
        ROS_ERROR("Failed to call service ParamSet");
        return std::make_tuple(false, 0, 0.0f);
    }
}

void MavrosBridge::poseImuCallback(const geometry_msgs::PoseStampedConstPtr& pose, const sensor_msgs::ImuConstPtr& imu)
{
    nav_msgs::Odometry odom;
    odom.header = pose->header;
    odom.pose.pose.position = pose->pose.position;
    odom.pose.pose.orientation = imu->orientation;
    odom.twist.twist.linear.x = imu->linear_acceleration.x;
    odom.twist.twist.linear.y = imu->linear_acceleration.y;
    odom.twist.twist.linear.z = imu->linear_acceleration.z;
    odom.twist.twist.angular.x = imu->angular_velocity.x;
    odom.twist.twist.angular.y = imu->angular_velocity.y;
    odom.twist.twist.angular.z = imu->angular_velocity.z;
    odom_pub_.publish(odom);
}

// void MavrosBridge::vioAlignTimerCallback(const ros::TimerEvent& event)
// {
//     std::tuple<bool, int, float> param = setParam("80", 2, 0.0f);
// }

void MavrosBridge::statusCallback(const swarm_msgs::SystemStatus msg)
{
    if(static_cast<int>(msg.drone_id) != self_id){
        return;
    }
}

void MavrosBridge::thermalTimerCallback(const ros::TimerEvent&)
{
    publishTemp("/sys/devices/virtual/thermal/thermal_zone0/temp", pub_temp0_);
    publishTemp("/sys/devices/virtual/thermal/thermal_zone1/temp", pub_temp1_);
}

void MavrosBridge::publishTemp(const std::string& file_path, ros::Publisher& publisher)
{
    std::ifstream file(file_path);
    if (file.is_open())
    {
        std::string line;
        if (std::getline(file, line))
        {
            try
            {
                float temp = std::stof(line) / 1000.0; // Convert milli-celsius to celsius
                std_msgs::Float32 msg;
                msg.data = temp;
                publisher.publish(msg);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM("Error converting temperature: " << e.what());
            }
        }
        file.close();
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open file: " << file_path);
    }
}

void MavrosBridge::rcCallback(mavros_msgs::RCIn msg)
{
    sensor_msgs::Joy joy = convertRCtoJoy(msg);
    joy_pub_.publish(joy);
}

bool MavrosBridge::activateCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    ROS_INFO("activate %u", req.data);
    res.success = activate(req.data);
    return true;
}

void MavrosBridge::pictStateTimerCallback(const ros::TimerEvent&)
{
    if(!nav_initialized_){
        transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "base_link"));
    }
    pubPictgramState(last_state_);
}

void MavrosBridge::pubPictgramState(mavros_msgs::State msg)
{
    std::string character;
    if (msg.connected){
        if (msg.mode == "GUIDED")
        {
            character = "fa-mendeley";
        }
        else
        {
            character = "fa-link";
        }
    }else{
        character = "three-dots";
    }
    
    jsk_rviz_plugins::Pictogram pict;
    pict.header = msg.header;
    pict.header.frame_id = "base_link";
    pict.pose.position.x = 0.0;
    pict.pose.position.y = 0.0;
    pict.pose.position.z = 0.1;
    pict.pose.orientation.x = 0.0;
    pict.pose.orientation.y = -0.7;
    pict.pose.orientation.z = 0.0;
    pict.pose.orientation.w = 0.7;
    pict.size = 0.1;
    
    if (msg.armed)
    {
        pict.color.r = 25 / 255.0;
        pict.color.g = 255 / 255.0;
        pict.color.b = 240 / 255.0;
    }
    else
    {
        pict.color.r = 150 / 255.0;
        pict.color.g = 150 / 255.0;
        pict.color.b = 150 / 255.0;
    }
    pict.action = jsk_rviz_plugins::Pictogram::ADD;
    pict.color.a = 1.0;
    pict.character = character.c_str();
    pict_state_pub_.publish(pict);
}

void MavrosBridge::stateCallback(mavros_msgs::State msg)
{
    last_state_ = msg;
    ap_connected_ = msg.connected;
    // pubPictgramState(msg);

    if (!ap_initialized_ && nav_initialized_)
    {
        initialSetup();
        ap_initialized_ = true;
    }
}

// void MavrosBridge::hpCallback(mavros_msgs::HomePosition msg)
// {
//     hp_valid_ = true;
// }

void MavrosBridge::batteryCallback(sensor_msgs::BatteryState msg)
{
    float voltage = msg.voltage;
    float min_voltage = 3.4 * cells_batt_;
    float max_voltage = 4.2 * cells_batt_;
    if (voltage < min_voltage){
        voltage = min_voltage;
    }
    if (voltage > max_voltage){
        voltage = max_voltage;
    }
    int battery = (voltage - min_voltage) / (max_voltage - min_voltage) * 100;
    std_msgs::Float32 battery_msg;
    battery_msg.data = battery;
    battery_pub_.publish(battery_msg);
}

void MavrosBridge::odomCallback(nav_msgs::Odometry msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    odom_cur_ = msg;
    // vision_pose_pub_.publish(pose);

    if(!nav_initialized_){
        ROS_INFO("Nav initialized");
        nav_initialized_ = true;
    }
}

void MavrosBridge::visionPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!nav_initialized_){
        ROS_INFO("Nav initialized");
        nav_initialized_ = true;
    }
}

bool MavrosBridge::checkMove(void)
{
    bool guided = last_state_.mode == "GUIDED";
    return last_state_.armed && guided && nav_initialized_;
}

sensor_msgs::Joy MavrosBridge::convertRCtoJoy(const mavros_msgs::RCIn& msg)
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

void MavrosBridge::setupStreamRate()
{
    ros::NodeHandle nh;
    ros::Rate rate(1);

    while (ros::ok() && !ap_connected_) {
        ROS_INFO("Waiting for ap_connected_");
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !set_msg_rate_client_.waitForExistence(ros::Duration(1.0))) {
        rate.sleep();
    }

    ros::Duration(3.0).sleep();

    mavros_msgs::MessageInterval msg_interval_srv;

    if (imu_freq_ != 0){
    ROS_INFO("Set IMU message rate to %d Hz", imu_freq_);
    msg_interval_srv.request.message_id = 27;
    msg_interval_srv.request.message_rate = imu_freq_;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
        ROS_ERROR("Failed to call service /mavros/set_message_interval");
        return;
    }
    }
    
    // mavros_msgs::MessageInterval msg_interval_srv;
    ROS_INFO("Set Battery message rate to %d Hz", infra_freq_);
    msg_interval_srv.request.message_id = 147;
    msg_interval_srv.request.message_rate = infra_freq_;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
        ROS_ERROR("Failed to call service /mavros/set_message_interval");
        return;
    }

    // mavros_msgs::MessageInterval msg_interval_srv;
    ROS_INFO("Set Local Position message rate to %d Hz", infra_freq_);
    msg_interval_srv.request.message_id = 32;
    msg_interval_srv.request.message_rate = infra_freq_;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
        ROS_ERROR("Failed to call service /mavros/set_message_interval");
        return;
    }

    ROS_INFO("Set Attitude message rate to %d Hz", infra_freq_);
    msg_interval_srv.request.message_id = 30;
    msg_interval_srv.request.message_rate = infra_freq_;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
        ROS_ERROR("Failed to call service /mavros/set_message_interval");
        return;
    }

    ROS_INFO("Set Attitude Quaternion message rate to %d Hz", infra_freq_);
    msg_interval_srv.request.message_id = 31;
    msg_interval_srv.request.message_rate = infra_freq_;
    if (!set_msg_rate_client_.call(msg_interval_srv)) {
        ROS_ERROR("Failed to call service /mavros/set_message_interval");
        return;
    }

    ROS_INFO("Stream rate setup complete !");
}

bool MavrosBridge::initGPIO()
{
    try {
        // GPIOを初期化
        GPIO::setmode(GPIO::BOARD);
        GPIO::setup(OUTPUT_PIN, GPIO::OUT);
        GPIO::PWM pwm(OUTPUT_PIN, 50); // 50Hz
        pwm.start(0); // デューティ比0%でスタート
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("GPIO initialization failed: %s", e.what());
        return false;
    }
}

void MavrosBridge::cleanupGPIO()
{
    try {
        GPIO::cleanup();
    } catch (const std::exception& e) {
        ROS_ERROR("GPIO cleanup failed: %s", e.what());
    }
}

bool MavrosBridge::rotateMotorCallback(hardware_utils::RotateMotor::Request& req,
                                      hardware_utils::RotateMotor::Response& res)
{
    try {
        GPIO::PWM pwm(OUTPUT_PIN, 50); // 50Hz
        
        // PWMのデューティ比を80%に設定
        pwm.ChangeDutyCycle(80);
        
        // 指定された時間待機
        ros::Duration(req.duration).sleep();
        
        // モーターを停止
        pwm.ChangeDutyCycle(0);
        
        res.success = true;
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Motor control failed: %s", e.what());
        res.success = false;
        return false;
    }
}

// デストラクタでGPIOをクリーンアップ
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