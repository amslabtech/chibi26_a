#ifndef obstacle_detector_HPP
#define obstacle_detector_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ObstacleDetector : public rclcpp::Node
{
    public:
        ObstacleDetector();
        //コールバック関数
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);//Lidarから障害物の情報を取得

        //関数
        void process();//一定周期で行う処理(obstacle_detectorの処理)
        void scan_obstacle();//Lidarから障害物情報を取得し，障害物の座標をpublish
        bool is_ignore_scan(int index);//無視するlidar情報の範囲の決定

        // 変数
        int hz_ = 10; // 制御周期
        double obs_dist = 1.0; // 障害物までの距離
        bool flag_scan_ = false;
        std::string robot_frame;
        std::optional<sensor_msgs::msg::LaserScan> laser_;

        //Pub & Sub & timer
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obs_pub_;
};

#endif  // b_obstacle_detector_HPP