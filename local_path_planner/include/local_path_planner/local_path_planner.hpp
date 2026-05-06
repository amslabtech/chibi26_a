/*
DWA: Dynamic Window Approach
*/

#ifndef LOCAL_PATH_PLANNER_A_HPP
#define LOCAL_PATH_PLANNER_A_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist.hpp"

// =========================
// 構造体定義
// =========================

struct State
{
    double x;        // [m]
    double y;        // [m]
    double yaw;      // [rad]
    double vx, vy; // [m/s]
    double yawrate;  // [rad/s]
};

struct DynamicWindow
{
    double min_vx, min_vy;     // [m/s]
    double max_vx, max_vy;     // [m/s]
    double min_yawrate; // [rad/s] 
    double max_yawrate; // [rad/s]
};

// ===================
// DWA_Plannerクラス
// =========================
class DWAPlanner : public rclcpp::Node
{
public:
    DWAPlanner(); 
    void process(); 
    int get_freq(); 

private:
    // コールバック関数
    void local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // その他の関数
    void   send_velocity(double vx, double vy, double yawrate);
    void   move(State& state, double vx, double vy, double yawrate);
    // void   visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now);
    double normalize_angle(double angle);
    double calc_evaluation(const std::vector<State>& traj, const geometry_msgs::msg::PoseArray& current_obs);
    double calc_heading_eval(const std::vector<State>& traj);
    double calc_dist_eval(const std::vector<State>& traj, const geometry_msgs::msg::PoseArray& obs_list);
    double calc_vel_eval(const std::vector<State>& traj);
    std::vector<State> calc_traj(double vx, double vy, double yawrate);

    void calc_dynamic_window();
    void change_mode();
    bool can_move();
    std::vector<double> calc_final_input(const geometry_msgs::msg::PoseArray& current_obs);
    void visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);

    // ----- パラメータ（変数はそのまま維持） -----
    int    hz_ ;
    double max_vel_y1_, max_vel_y2_, max_vel_y_;
    double max_vel_, max_vel1_, max_vel2_;
    double min_vel_, max_yawrate_, max_yawrate1_, max_yawrate2_;
    double max_accel_, max_dyawrate_, v_reso_, vy_reso_, yawrate_reso_;
    double dt_, predict_time_, predict_time1_, predict_time2_;
    double robot_radius_, radius_margin1_;
    double goal_tolerance_, search_range_;

    bool flag_local_goal_ = false;
    bool flag_obs_poses_  = false;

    std::vector<double> mode_log_;
    double weight_heading_, weight_heading1_;
    double weight_dist_, weight_dist1_;
    double weight_vel_, score_dist;
    
    rclcpp::Clock clock_;
    
    State robot_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    DynamicWindow dw_;

    // Subscriber / Publisher
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_local_goal_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obs_poses_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimal_path_;

    geometry_msgs::msg::PointStamped local_goal_;
    geometry_msgs::msg::PoseArray    obs_poses_;
    geometry_msgs::msg::PointStamped goal_msg_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

#endif