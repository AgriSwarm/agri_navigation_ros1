#include "hard_mavros_bridge.h"

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
    // try {
    //     ROS_INFO("Rotate motor for %f seconds", req.duration);
    //     GPIO::PWM pwm(OUTPUT_PIN, 50); // 50Hz
        
    //     // PWMのデューティ比を80%に設定
    //     pwm.ChangeDutyCycle(80);
        
    //     // 指定された時間待機
    //     ros::Duration(req.duration).sleep();
        
    //     // モーターを停止
    //     pwm.ChangeDutyCycle(0);
        
    //     res.success = true;
    //     return true;
    // } catch (const std::exception& e) {
    //     ROS_ERROR("Motor control failed: %s", e.what());
    //     res.success = false;
    //     return false;
    // }

    // バッテリー電圧に応じてモーターの回転速度を調整
    float rated_voltage = 5.0;
    if (batt_voltage_ <= rated_voltage) {
        ROS_ERROR("Battery voltage is not available");
        res.success = false;
        return false;
    }
    float duty_cycle = rated_voltage / batt_voltage_ * 100;

    try {
        // Pythonスクリプトのパスを指定
        std::string script_path = ros::package::getPath("hardware_utils") + "/scripts/motor_control.py";
        
        // コマンドを構築
        std::string command = "python3 " + script_path + " " + std::to_string(req.duration) + " " + std::to_string(duty_cycle);
        
        // Pythonスクリプトを実行
        int result = system(command.c_str());
        std_msgs::ColorRGBA g_cone_color;
        g_cone_color.a = 0.6;
        
        if (result == 0) {
            ROS_INFO("Motor rotation completed successfully");
            res.success = true;
            g_cone_color.r = 1.0;
            g_cone_color.g = 1.0;
            g_cone_color.b = 0.0;
            pubShotCone(g_cone_color, req.duration);
        } else {
            ROS_ERROR("Failed to rotate motor");
            res.success = false;
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Motor control failed: %s", e.what());
        res.success = false;
        return false;
    }
}
