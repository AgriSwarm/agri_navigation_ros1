#include "hard_mavros_bridge.h"

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