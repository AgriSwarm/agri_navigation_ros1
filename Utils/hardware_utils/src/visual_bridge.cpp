#include "hard_mavros_bridge.h"

void MavrosBridge::publishCpuUsage(const std::string& file_path, ros::Publisher& publisher)
{
    std::ifstream file(file_path);
    if (file.is_open())
    {
        std::string line;
        if (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string cpuLabel;
            unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
            // /proc/statの1行目は "cpu  <user> <nice> <system> <idle> <iowait> <irq> <softirq> <steal> ..."
            if (iss >> cpuLabel >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal)
            {
                // アイデル時間はidleとiowaitの和
                unsigned long long idleTime = idle + iowait;
                // 非アイデル時間はその他の値の和
                unsigned long long nonIdle = user + nice + system + irq + softirq + steal;
                unsigned long long total = idleTime + nonIdle;

                // 前回値をstatic変数で保持（初回は0のため計算できない）
                static unsigned long long prevTotal = 0, prevIdle = 0;
                float cpuUsage = 0.0f;
                if (prevTotal != 0)
                {
                    unsigned long long totalDelta = total - prevTotal;
                    unsigned long long idleDelta = idleTime - prevIdle;
                    if (totalDelta > 0)
                    {
                        // 使用率＝(デルタ合計－デルタアイドル)／デルタ合計×100[%]
                        cpuUsage = static_cast<float>(totalDelta - idleDelta) / totalDelta * 100.0f;
                    }
                }
                prevTotal = total;
                prevIdle  = idleTime;

                std_msgs::Float32 msg;
                msg.data = cpuUsage;
                publisher.publish(msg);
            }
            else
            {
                ROS_ERROR_STREAM("Failed to parse CPU stat line: " << line);
            }
        }
        file.close();
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open file: " << file_path);
    }
}

// GPU使用率を読み出してpublishする関数
void MavrosBridge::publishGpuUsage(const std::string& file_path, ros::Publisher& publisher)
{
    std::ifstream file(file_path);
    if (file.is_open())
    {
        std::string line;
        if (std::getline(file, line))
        {
            try
            {
                // GPUのloadファイルは、例えば"674"のような値が得られると仮定し、
                // 必要に応じて10で割ることでパーセンテージ(67.4%)に変換する例です。
                float gpuLoad = std::stof(line) / 10.0f;
                std_msgs::Float32 msg;
                msg.data = gpuLoad;
                publisher.publish(msg);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM("Error converting GPU load: " << e.what());
            }
        }
        file.close();
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open file: " << file_path);
    }
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

void MavrosBridge::pubDebugText(swarm_msgs::SystemStatus msg)
{
    jsk_rviz_plugins::OverlayText text;
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.width = 500;
    text.height = 500;
    text.left = 10;
    text.top = 10;
    text.bg_color.r = 0.0;
    text.bg_color.g = 0.0;
    text.bg_color.b = 0.0;
    text.bg_color.a = 0.5;
    text.line_width = 2;
    text.text_size = 20;
    text.font = "DejaVu Sans Mono";
    text.fg_color.r = 1.0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;

    std::string infra_status;
    switch(msg.infra_status) {
        case swarm_msgs::SystemStatus::INFRA_INACTIVE:
            infra_status = "INACTIVE";
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_CONNECTED:
            infra_status = "AP_CONNECTED";
            break;
        case swarm_msgs::SystemStatus::INFRA_ODOM_READY:
            infra_status = "ODOM_READY";
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_EKF_READY:
            infra_status = "AP_EKF_READY";
            break;
        case swarm_msgs::SystemStatus::INFRA_ARMED:
            infra_status = "ARMED";
            break;
    }

    // std::string status_str = msg.ap_status + "::" + msg.nav_status;
    std::string status_str = "INFRA: " + infra_status + "\nAP: " + msg.ap_status + "\nNAV: " + msg.nav_status;
    text.text = status_str;

    debug_text_pub_.publish(text);
}

void MavrosBridge::pubPictgramState(swarm_msgs::SystemStatus msg)
{
    jsk_rviz_plugins::PictogramArray pictogram_array;
    pictogram_array.header = msg.header;
    pictogram_array.header.frame_id = "base_link";

    // ステータス文字列の結合と分割
    std::string status_str = msg.ap_status + "::" + msg.nav_status;
    std::vector<std::string> chunks;
    for (size_t i = 0; i < status_str.length(); i += 2) {
        if (i + 1 < status_str.length()) {
            chunks.push_back(status_str.substr(i, 2));
        } else {
            chunks.push_back(status_str.substr(i, 1) + " ");
        }
    }

    // 各文字のPictogramを作成
    float horizontal_offset = -0.02f * (chunks.size() - 1) / 2.0f;
    for (size_t i = 0; i < chunks.size(); i++) {
        jsk_rviz_plugins::Pictogram status_pic;
        status_pic.header = msg.header;
        status_pic.header.frame_id = "base_link";
        status_pic.pose.position.x = horizontal_offset + 0.02f * i;
        status_pic.pose.position.y = 0.0;
        status_pic.pose.position.z = 0.15; // Offset for upper text
        status_pic.pose.orientation.x = 0.0;
        status_pic.pose.orientation.y = -0.7;
        status_pic.pose.orientation.z = 0.0;
        status_pic.pose.orientation.w = 0.7;
        status_pic.mode = jsk_rviz_plugins::Pictogram::STRING_MODE;
        status_pic.character = chunks[chunks.size() - i - 1];
        status_pic.size = 0.05;
        status_pic.ttl = 0.0;
        status_pic.action = jsk_rviz_plugins::Pictogram::ADD;

        // Set color based on status
        status_pic.color.r = 25 / 255.0;
        status_pic.color.g = 255 / 255.0;
        status_pic.color.b = 240 / 255.0;
        status_pic.color.a = 1.0;

        pictogram_array.pictograms.push_back(status_pic);
    }

    // Infra status pictogram
    jsk_rviz_plugins::Pictogram infra_pic;
    infra_pic.header = msg.header;
    infra_pic.header.frame_id = "base_link";
    infra_pic.pose.position.x = 0.0;
    infra_pic.pose.position.y = 0.0;
    infra_pic.pose.position.z = 0.1; // Offset for lower icon
    infra_pic.pose.orientation.x = 0.0;
    infra_pic.pose.orientation.y = -0.7;
    infra_pic.pose.orientation.z = 0.0;
    infra_pic.pose.orientation.w = 0.7;
    infra_pic.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
    
    // Set pictogram character based on infra_status
    switch(msg.infra_status) {
        case swarm_msgs::SystemStatus::INFRA_INACTIVE:
            infra_pic.character = "three-dots";
            infra_pic.color.r = 150 / 255.0;
            infra_pic.color.g = 150 / 255.0;
            infra_pic.color.b = 150 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_CONNECTED:
            infra_pic.character = "fa-link";
            infra_pic.color.r = 150 / 255.0;
            infra_pic.color.g = 150 / 255.0;
            infra_pic.color.b = 150 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_ODOM_READY:
            infra_pic.character = "fa-mendeley";
            infra_pic.color.r = 150 / 255.0;
            infra_pic.color.g = 150 / 255.0;
            infra_pic.color.b = 150 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_AP_EKF_READY:
            infra_pic.character = "fa-mendeley";
            infra_pic.color.r = 25 / 255.0;
            infra_pic.color.g = 255 / 255.0;
            infra_pic.color.b = 240 / 255.0;
            break;
        case swarm_msgs::SystemStatus::INFRA_ARMED:
            infra_pic.character = "fa-rocket";
            infra_pic.color.r = 25 / 255.0;
            infra_pic.color.g = 255 / 255.0;
            infra_pic.color.b = 240 / 255.0;
            break;
    }
    
    infra_pic.color.a = 1.0;
    infra_pic.size = 0.1;
    infra_pic.ttl = 0.0;
    infra_pic.action = jsk_rviz_plugins::Pictogram::ADD;

    pictogram_array.pictograms.push_back(infra_pic);

    pict_state_pub_.publish(pictogram_array);
}

void MavrosBridge::pubShotCone(rviz_visual_tools::colors cone_color, double duration)
{
    geometry_msgs::Pose cone_pose;
    cone_pose.position.x = 0.0;
    cone_pose.position.y = 0.0;
    cone_pose.position.z = 1.0;
    cone_pose.orientation.x = 0.0;
    cone_pose.orientation.y = 0.0;
    cone_pose.orientation.z = 0.0;
    cone_pose.orientation.w = 1.0;

    double cone_angle = - M_PI / 2;
    // 修正：直接 cone_color を渡す（第三引数は rviz_visual_tools::colors 型）
    visual_tools_->setLifetime(duration);
    visual_tools_->publishCone(cone_pose, cone_angle, cone_color, 1);
    visual_tools_->trigger();
}
