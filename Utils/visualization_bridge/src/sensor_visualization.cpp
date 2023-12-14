#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <random>
#include <array>
#include <boost/bind.hpp>

class SensorVisualization {
public:
    SensorVisualization() {
        // Initialize ROS node handle
        ros::NodeHandle nh("~");

        // read param
        bool display_objects;
        int drones_num;
        nh.param<bool>("display_objects", display_objects, false);
        nh.param<int>("drones_num", drones_num, 1);
        
        // Initialize visual tools
        visual_tools_apple_.reset(new rviz_visual_tools::RvizVisualTools("world", "/apple"));
        visual_tools_apple_->loadMarkerPub();
        visual_tools_apple_->setLifetime(1);
        for (int i = 0; i < drones_num; ++i) {
            visual_tools_sensor_[i].reset(new rviz_visual_tools::RvizVisualTools("world", "/sensor_" + std::to_string(i)));
            visual_tools_sensor_[i]->loadMarkerPub();
            visual_tools_sensor_[i]->setLifetime(1);
        }
        // Subscriber to odometry
        for (int i = 0; i < drones_num; ++i) {
            odom_sub_[i] = nh.subscribe<nav_msgs::Odometry>(
                "/drone_" + std::to_string(i) + "_visual_slam/odom", 
                1, 
                boost::bind(&SensorVisualization::odomCallback, this, boost::placeholders::_1, i)
            );
        }
        
        // Timer for publishing spheres
        if (display_objects){
            timer_ = nh.createTimer(ros::Duration(1), &SensorVisualization::timerCallback, this);
        }

        // Setup random engine
        rand_x_ = std::uniform_real_distribution<double>(-10.0, 10.0);
        rand_y_ = std::uniform_real_distribution<double>(-10.0, 10.0);
        rand_z_ = std::uniform_real_distribution<double>(1.8, 2.1);
        rand_engine_.seed(std::random_device{}());
        distribution_ = std::uniform_real_distribution<double>(-10.0, 10.0);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int i) {
        // last_pose_ = msg->pose.pose;
        visual_tools_sensor_[i]->deleteAllMarkers();
        publishSensorZone(msg->pose.pose, i);
    }
    
    void publishSensorZone(geometry_msgs::Pose pose, int i) {
        // Create Cone Marker at the position above the robot
        geometry_msgs::Pose pose_;
        pose_.position.x = pose.position.x;
        pose_.position.y = pose.position.y;
        pose_.position.z = pose.position.z + 0.1; // Cone over the robot
        // pose.orientation = msg->pose.pose.orientation;     // Same orientation as the robot
        // base座標系で回転させる
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        // Eigen::Quaterniond q_rot = q * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q_rot = q * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

        pose_.orientation.w = q_rot.w();
        pose_.orientation.x = q_rot.x();
        pose_.orientation.y = q_rot.y();
        pose_.orientation.z = q_rot.z();

        // Determine the angle for the base of the cone
        double angle = M_PI / 2; // Replace with desired angle, if needed
        visual_tools_sensor_[i]->publishCone(pose_, angle, rviz_visual_tools::TRANSLUCENT_DARK, 2);
        visual_tools_sensor_[i]->trigger();
    }

    void timerCallback(const ros::TimerEvent&) {
        std::vector<geometry_msgs::Point> points;
        double scale_x, scale_y, scale_z;
        scale_x = scale_y = 2.0 * 4.8/100;
        scale_z = 2.0 * 6.0/100;
        
        for (int i = 0; i < 1000; ++i) {
            geometry_msgs::Point point;
            point.x = rand_x_(rand_engine_);
            point.y = rand_y_(rand_engine_);
            point.z = rand_z_(rand_engine_);
            points.push_back(point);
        }

        visual_tools_apple_->publishSpheres(points, rviz_visual_tools::colors::ORANGE, scale_x);
        visual_tools_apple_->trigger();
    }

private:
    static constexpr int DEFAULT_DRONES_NUM = 10;
    // ros::Subscriber odom_sub_;
    std::array<ros::Subscriber, DEFAULT_DRONES_NUM> odom_sub_;
    ros::Timer timer_;
    std::unique_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_apple_;
    // std::unique_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_sensor_;
    std::array<std::unique_ptr<rviz_visual_tools::RvizVisualTools>, DEFAULT_DRONES_NUM> visual_tools_sensor_;

    std::mt19937 rand_engine_;
    geometry_msgs::Pose last_pose_;
    std::uniform_real_distribution<double> rand_x_;
    std::uniform_real_distribution<double> rand_y_;
    std::uniform_real_distribution<double> rand_z_;
    std::uniform_real_distribution<double> distribution_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_visualization_node");
    SensorVisualization visualizer;

    ros::spin();

    return 0;
}