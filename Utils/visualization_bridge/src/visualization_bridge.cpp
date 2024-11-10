#include <ros/ros.h>
#include <swarm_msgs/SystemStatus.h>
#include <swarm_msgs/IndexedOdometry.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <map>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Dense>

class VisualizationBridge {
public:
    VisualizationBridge() {
        ros::NodeHandle nh("~");
        
        // Subscribers
        system_status_sub = nh.subscribe("/hardware_bridge/system_status", 1, 
            &VisualizationBridge::SystemStatusCallback, this);
        indexed_odom_sub = nh.subscribe("/d2vins/indexed_odometry", 1, 
            &VisualizationBridge::IndexedOdometryCallback, this);
        
        // Publishers
        pictogram_array_pub = nh.advertise<jsk_rviz_plugins::PictogramArray>("drone_status_viz", 1);

        // pictgram_timer = nh.createTimer(ros::Duration(0.1), &VisualizationBridge::timerCallback, this);
    }

private:
    ros::Subscriber system_status_sub;
    ros::Subscriber indexed_odom_sub;
    ros::Publisher pictogram_array_pub;
    ros::Timer pictgram_timer;
    
    std::map<uint32_t, swarm_msgs::SystemStatus> drone_status_map;
    std::map<uint32_t, geometry_msgs::Pose> drone_pose_map;

    void SystemStatusCallback(const swarm_msgs::SystemStatus::ConstPtr& msg) {
        drone_status_map[msg->drone_id] = *msg;
    }

    void IndexedOdometryCallback(const swarm_msgs::IndexedOdometry::ConstPtr& msg) {
        drone_pose_map[msg->drone_id] = msg->odom.pose.pose;
        PublishPictogram(msg->drone_id);
    }

    // void timerCallback(const ros::TimerEvent& event) {
    //     // dummy system status
    //     swarm_msgs::SystemStatus status;
    //     status.header.stamp = ros::Time::now();
    //     status.header.frame_id = "world";
    //     status.drone_id = 0;
    //     status.ap_status = "AP_STATUS";
    //     status.nav_status = "NAV_STATUS";
    //     status.infra_status = swarm_msgs::SystemStatus::INFRA_ARMED;
    //     drone_status_map[0] = status;
    //     PublishPictogram(0);
    // }

    private:
    // 4x4の変換行列を作成する関数
    Eigen::Matrix4d createTransformMatrix(const geometry_msgs::Pose& pose) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        
        // 回転部分
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, 
                           pose.orientation.y, pose.orientation.z);
        transform.block<3,3>(0,0) = q.toRotationMatrix();
        
        // 並進部分
        transform(0,3) = pose.position.x;
        transform(1,3) = pose.position.y;
        transform(2,3) = pose.position.z;
        
        return transform;
    }

    // 位置とクォータニオンから姿勢を設定する関数
    void setPoseFromTransform(geometry_msgs::Pose& pose, const Eigen::Matrix4d& transform) {
        // 位置の設定
        pose.position.x = transform(0,3);
        pose.position.y = transform(1,3);
        pose.position.z = transform(2,3);
        
        // 回転の設定
        Eigen::Matrix3d rot = transform.block<3,3>(0,0);
        Eigen::Quaterniond q(rot);
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
    }

    void PublishPictogram(uint32_t drone_id) {
        if (drone_status_map.find(drone_id) == drone_status_map.end()) return;

        auto status = drone_status_map[drone_id];
        auto base_pose = drone_pose_map[drone_id];

        jsk_rviz_plugins::PictogramArray pictogram_array;
        pictogram_array.header = status.header;
        pictogram_array.header.frame_id = "world";

        // ベースとなる変換行列を作成
        Eigen::Matrix4d base_transform = createTransformMatrix(base_pose);

        // ステータス文字列の処理
        std::string status_str = status.ap_status + "::" + status.nav_status;
        std::vector<std::string> chunks;
        for (size_t i = 0; i < status_str.length(); i += 2) {
            if (i + 1 < status_str.length()) {
                chunks.push_back(status_str.substr(i, 2));
            } else {
                chunks.push_back(status_str.substr(i, 1) + " ");
            }
        }

        // 文字のPictogramを作成
        float horizontal_offset = -0.02f * (chunks.size() - 1) / 2.0f;
        for (size_t i = 0; i < chunks.size(); i++) {
            jsk_rviz_plugins::Pictogram status_pic;
            status_pic.header = status.header;
            status_pic.header.frame_id = "world";

            // 文字用の追加の変換行列
            Eigen::Matrix4d text_transform = Eigen::Matrix4d::Identity();
            text_transform(1,3) = horizontal_offset + 0.02f * i;
            text_transform(2,3) = 0.15;  // Z方向のオフセット
            
            // テキストの向きを設定
            Eigen::Quaterniond text_rotation(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()));
            text_transform.block<3,3>(0,0) = text_rotation.toRotationMatrix();

            // 最終的な変換行列を計算
            Eigen::Matrix4d final_transform = base_transform * text_transform;
            
            // 変換行列からPoseを設定
            setPoseFromTransform(status_pic.pose, final_transform);

            status_pic.mode = jsk_rviz_plugins::Pictogram::STRING_MODE;
            status_pic.character = chunks[chunks.size() - i - 1];
            status_pic.size = 0.05;
            status_pic.ttl = 0.0;
            status_pic.action = jsk_rviz_plugins::Pictogram::ADD;
            status_pic.color.r = 25 / 255.0;
            status_pic.color.g = 255 / 255.0;
            status_pic.color.b = 240 / 255.0;
            status_pic.color.a = 1.0;

            pictogram_array.pictograms.push_back(status_pic);
        }

        // Infra status pictogram
        jsk_rviz_plugins::Pictogram infra_pic;
        infra_pic.header = status.header;
        infra_pic.header.frame_id = "world";

        // インフラアイコン用の追加の変換行列
        Eigen::Matrix4d infra_transform = Eigen::Matrix4d::Identity();
        infra_transform(2,3) = 0.1;  // Z方向のオフセット
        
        // インフラアイコンの向きを設定
        Eigen::Quaterniond infra_rotation(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()));
        infra_transform.block<3,3>(0,0) = infra_rotation.toRotationMatrix();

        // 最終的な変換行列を計算
        Eigen::Matrix4d final_infra_transform = base_transform * infra_transform;
        
        // 変換行列からPoseを設定
        setPoseFromTransform(infra_pic.pose, final_infra_transform);

        infra_pic.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
        
        // Set pictogram character based on infra_status
        switch(status.infra_status) {
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
        pictogram_array_pub.publish(pictogram_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_bridge_node");
    VisualizationBridge visualizer;

    ros::spin();

    return 0;
}