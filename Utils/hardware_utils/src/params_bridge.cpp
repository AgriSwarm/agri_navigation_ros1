#include "hard_mavros_bridge.h"

// gen.add("VISO_DELAY_MS", int_t, 0, "VISO_DELAY_MS", 0, 0, 250)
// gen.add("EK3_POSNE_M_NSE", double_t, 0, "EK3_POSNE_M_NSE", 0.5, 0.1, 10.0)
// gen.add("EK3_ALT_M_NSE", double_t, 0, "EK3_ALT_M_NSE", 0.5, 0.1, 100.0)
// gen.add("EK3_YAW_M_NSE", double_t, 0, "EK3_YAW_M_NSE", 0.5, 0.05, 1.0)
// gen.add("VISO_POS_M_NSE", double_t, 0, "VISO_POS_M_NSE", 0.5, 0.1, 10.0)
// gen.add("VISO_YAW_M_NSE", double_t, 0, "VISO_YAW_M_NSE", 0.5, 0.05, 1.0)
// gen.add("EK3_POS_I_GATE", int_t, 0, "EK3_POS_I_GATE", 1000, 100, 1000)
// gen.add("EK3_YAW_I_GATE", int_t, 0, "EK3_YAW_I_GATE", 1000, 100, 1000)
// gen.add("EK3_VELD_M_NSE", double_t, 0, "EK3_VELD_M_NSE", 0.5, 0.05, 5.0)
// gen.add("EK3_VELNE_M_NSE", double_t, 0, "EK3_VELNE_M_NSE", 0.5, 0.05, 5.0)
// gen.add("EK3_ACC_P_NSE", double_t, 0, "EK3_ACC_P_NSE", 0.5, 0.01, 1.0)

const std::vector<MavrosBridge::ParamPair> MavrosBridge::params = {
    {"ATC_ANG_RLL_P", &hardware_utils::PIDConfig::ATC_ANG_RLL_P, false},
    {"ATC_ANG_PIT_P", &hardware_utils::PIDConfig::ATC_ANG_PIT_P, false},
    {"ATC_ANG_YAW_P", &hardware_utils::PIDConfig::ATC_ANG_YAW_P, false},
    {"ATC_RAT_RLL_P", &hardware_utils::PIDConfig::ATC_RAT_RLL_P, false},
    {"ATC_RAT_RLL_I", &hardware_utils::PIDConfig::ATC_RAT_RLL_I, false},
    {"ATC_RAT_RLL_D", &hardware_utils::PIDConfig::ATC_RAT_RLL_D, false},
    {"ATC_RAT_PIT_P", &hardware_utils::PIDConfig::ATC_RAT_PIT_P, false},
    {"ATC_RAT_PIT_I", &hardware_utils::PIDConfig::ATC_RAT_PIT_I, false},
    {"ATC_RAT_PIT_D", &hardware_utils::PIDConfig::ATC_RAT_PIT_D, false},
    {"ATC_RAT_YAW_P", &hardware_utils::PIDConfig::ATC_RAT_YAW_P, false},
    {"ATC_RAT_YAW_I", &hardware_utils::PIDConfig::ATC_RAT_YAW_I, false},
    {"ATC_RAT_YAW_D", &hardware_utils::PIDConfig::ATC_RAT_YAW_D, false},
    {"PSC_POSXY_P", &hardware_utils::PIDConfig::PSC_POSXY_P, false},
    {"PSC_POSZ_P", &hardware_utils::PIDConfig::PSC_POSZ_P, false},
    {"PSC_VELXY_P", &hardware_utils::PIDConfig::PSC_VELXY_P, false},
    {"PSC_VELXY_I", &hardware_utils::PIDConfig::PSC_VELXY_I, false},
    {"PSC_VELZ_P", &hardware_utils::PIDConfig::PSC_VELZ_P, false},
    {"PSC_VELZ_I", &hardware_utils::PIDConfig::PSC_VELZ_I, false},
    {"PSC_ACCZ_P", &hardware_utils::PIDConfig::PSC_ACCZ_P, false},
    {"PSC_ACCZ_I", &hardware_utils::PIDConfig::PSC_ACCZ_I, false},
    {"PSC_ACCZ_D", &hardware_utils::PIDConfig::PSC_ACCZ_D, false},
    {"VISO_DELAY_MS", &hardware_utils::PIDConfig::VISO_DELAY_MS, true}, // integer
    {"EK3_POSNE_M_NSE", &hardware_utils::PIDConfig::EK3_POSNE_M_NSE, false},
    {"EK3_ALT_M_NSE", &hardware_utils::PIDConfig::EK3_ALT_M_NSE, false},
    {"EK3_YAW_M_NSE", &hardware_utils::PIDConfig::EK3_YAW_M_NSE, false},
    {"VISO_POS_M_NSE", &hardware_utils::PIDConfig::VISO_POS_M_NSE, false},
    {"VISO_YAW_M_NSE", &hardware_utils::PIDConfig::VISO_YAW_M_NSE, false},
    {"EK3_POS_I_GATE", &hardware_utils::PIDConfig::EK3_POS_I_GATE, true}, // integer
    {"EK3_YAW_I_GATE", &hardware_utils::PIDConfig::EK3_YAW_I_GATE, true}, // integer
    {"EK3_VELD_M_NSE", &hardware_utils::PIDConfig::EK3_VELD_M_NSE, false},
    {"EK3_VELNE_M_NSE", &hardware_utils::PIDConfig::EK3_VELNE_M_NSE, false},
    {"EK3_ACC_P_NSE", &hardware_utils::PIDConfig::EK3_ACC_P_NSE, false},
    {"EK3_GYRO_P_NSE", &hardware_utils::PIDConfig::EK3_GYRO_P_NSE, false},
};

void MavrosBridge::configCallback(hardware_utils::PIDConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure Request:");

    for (const auto& param : params) {
        if (config.*(param.value) != config_last_.*(param.value)) {
            if (param.integer)
                setParam(param.name, static_cast<int>(config.*(param.value)), 0);
            else
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
    // ros::Duration(2.0).sleep();

    pullAndSetParams();

    for (const auto& param : params) {
        auto result = getParam(param.name);
        bool success = std::get<0>(result);
        if (success) {
            if (param.integer)
                config.*(param.value) = static_cast<double>(std::get<1>(result));
            else
                config.*(param.value) = std::get<2>(result);
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
        ROS_ERROR("Failed to call service ParamGet, retrying...");
        ros::Duration(1.0).sleep();
        return getParam(param);
        // return std::make_tuple(false, 0, 0.0f);
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
            try {
                YAML::Node config = YAML::LoadFile(yaml_path_);
                // config[param] = value_real;
                if (value_integer != 0)
                    config[param] = value_integer;
                else
                    config[param] = value_real;
                std::ofstream fout(yaml_path_);
                fout << config;
                fout.close();
                
                ROS_INFO("Updated YAML file with new parameter value");
            } catch (const YAML::Exception& e) {
                ROS_ERROR("Failed to update YAML file: %s", e.what());
            }
        }
        else
        {
            ROS_ERROR("Failed to set %s", param.c_str());
        }
        return std::make_tuple(srv.response.success, 
                                srv.response.value.integer, 
                                srv.response.value.real);
    } else {
        ROS_ERROR("Failed to call service ParamSet, retrying...");
        ros::Duration(1.0).sleep();
        return setParam(param, value_integer, value_real);
        // return std::make_tuple(false, 0, 0.0f);
    }
}

void MavrosBridge::setupStreamRate()
{
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

void MavrosBridge::getResourcePath()
{
    // ROSパッケージのパスを取得
    std::string package_path;
    try {
        package_path = ros::package::getPath("agri_resources");
    } catch (ros::Exception& e) {
        ROS_ERROR("Failed to get package path: %s", e.what());
        return;
    }
    
    // configファイルのパスを構築
    yaml_path_ = package_path + "/config/ardupilot/" + ap_param_type_ + ".yaml";
    ROS_INFO("YAML path: %s", yaml_path_.c_str());
}

void MavrosBridge::pullAndSetParams()
{
    // パッケージパスの取得
    getResourcePath();

    // Git pull
    // std::string cmd = "cd " + ros::package::getPath("agri_resources") + "/config/ardupilot && git pull origin main";
    // int ret = system(cmd.c_str());
    // if (ret == -1) {
    //     ROS_ERROR("Failed to execute command: %s", strerror(errno));
    //     return;
    // } else {
    //     int exit_status = WEXITSTATUS(ret);
    //     if (exit_status != 0) {
    //         ROS_ERROR("Git pull command failed with exit status: %d", exit_status);
    //         return;
    //     }
    // }

    try {
        YAML::Node config = YAML::LoadFile(yaml_path_);
        
        // パラメータの設定
        for (const auto& param : params) {
            if (config[param.name]) {
                // float value = config[param.name].as<float>();
                // setParam(param.name, 0, value);
                if (param.integer) {
                    int value = config[param.name].as<int>();
                    setParam(param.name, value, 0);
                    ROS_INFO("Set parameter %s to %d", param.name.c_str(), value);
                } else {
                    float value = config[param.name].as<float>();
                    setParam(param.name, 0, value);
                    ROS_INFO("Set parameter %s to %f", param.name.c_str(), value);
                }
            }
        }
    } catch (YAML::Exception& e) {
        ROS_ERROR("Failed to load YAML file: %s", e.what());
    }
}