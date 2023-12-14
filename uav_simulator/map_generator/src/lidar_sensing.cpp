#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <deque>

class LidarCloudAccumulator {
public:
    LidarCloudAccumulator(){
        ros::NodeHandle nh("~");

        nh.param("window_time", window_time_, 10.0);
        
        // Subscriberの作成
        lidar_sub_ = nh.subscribe("/livox/lidar", 1, &LidarCloudAccumulator::lidarCallback, this);
      
        // Publisherの作成
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

        // Timerの作成
        timer_ = nh.createTimer(ros::Duration(0.1), &LidarCloudAccumulator::timerCallback, this);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // 現在のクラウドをタイムスタンプ付きで蓄積
        cloud_queue_.push_back(*cloud_msg);

        // 古いデータを削除
        while (!cloud_queue_.empty()) {
            const ros::Time cloud_time = cloud_queue_.front().header.stamp;
            if (ros::Time::now() - cloud_time > ros::Duration(window_time_)) {
                cloud_queue_.pop_front();
            } else {
                break;
            }
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        pcl::PointCloud<pcl::PointXYZ> accumulated_cloud;
        
        // 蓄積されたデータから点群を組み立てる
        for (const auto& cloud_msg : cloud_queue_) {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(cloud_msg, cloud);
            accumulated_cloud += cloud;
        }

        // データが存在しない場合は何もしない
        if (accumulated_cloud.empty()) {
            // ROS_INFO("No data to publish");
            return;
        }

        // ROS_INFO("Publishing accumulated cloud");

        // マージされた点群をパブリッシュする
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(accumulated_cloud, output);
        output.header.frame_id = "world";
        output.header.stamp = ros::Time::now();

        cloud_pub_.publish(output);
    }

private:
    ros::Subscriber lidar_sub_;
    ros::Publisher cloud_pub_;
    ros::Timer timer_;
    double window_time_;
    std::deque<sensor_msgs::PointCloud2> cloud_queue_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_cloud_accumulator");
  
    // if (argc != 2) {
    //     ROS_ERROR("Usage: lidar_cloud_accumulator <window_time_in_seconds>");
    //     return 1;
    // }
  
    // double window_time = std::stod(argv[1]);

    LidarCloudAccumulator accumulator;
    ros::spin();
  
    return 0;
}