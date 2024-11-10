#include "hard_mavros_bridge.h"

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