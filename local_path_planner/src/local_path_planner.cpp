#include "local_path_planner/local_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言と取得 ######
    this->declare_parameter("hz", 10);
    this->declare_parameter("max_vel1", 0.5); 
    this->declare_parameter("max_vel_y1", 0.3);
    this->declare_parameter("max_vel2", 0.2);   
    this->declare_parameter("max_vel_y2", 0.1);
    this->declare_parameter("max_yawrate1", 1.0);
    this->declare_parameter("max_yawrate2", 0.5);
    this->declare_parameter("max_accel", 2.0);
    this->declare_parameter("min_vel", 0.0);
    this->declare_parameter("max_dyawrate", 2.0);
    this->declare_parameter("predict_time1", 2.0);
    this->declare_parameter("predict_time2", 1.2);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("weight_heading", 0.6);
    this->declare_parameter("weight_dist", 0.2);
    this->declare_parameter("weight_vel", 0.2);
    this->declare_parameter("roomba_radius", 0.3);
    this->declare_parameter("vel_reso", 0.01);
    this->declare_parameter("yawrate_reso", 0.05);
    this->declare_parameter("search_range", 3.0);

    hz_ = this->get_parameter("hz").as_int();
    max_vel1_ = this->get_parameter("max_vel1").as_double();
    max_vel2_ = this->get_parameter("max_vel2").as_double();
    max_vel_y1_ = this->get_parameter("max_vel_y1").as_double(); 
    max_vel_y2_ = this->get_parameter("max_vel_y2").as_double(); 
    max_accel_ = this->get_parameter("max_accel").as_double();
    max_dyawrate_ = this->get_parameter("max_dyawrate").as_double();
    v_reso_ = this->get_parameter("vel_reso").as_double();
    vy_reso_ = v_reso_ * 2.0;
    yawrate_reso_ = this->get_parameter("yawrate_reso").as_double();
    robot_radius_ = this->get_parameter("roomba_radius").as_double();
    search_range_ = this->get_parameter("search_range").as_double();
    max_yawrate1_ = this->get_parameter("max_yawrate1").as_double();
    max_yawrate2_ = this->get_parameter("max_yawrate2").as_double();
    predict_time1_ = this->get_parameter("predict_time1").as_double();
    predict_time2_ = this->get_parameter("predict_time2").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    weight_heading1_ = this->get_parameter("weight_heading").as_double();
    weight_dist1_ = this->get_parameter("weight_dist").as_double();
    weight_vel_ = this->get_parameter("weight_vel").as_double();

    // 初期モード設定
    max_vel_ = max_vel1_;
    max_yawrate_ = max_yawrate1_;
    weight_heading_ = weight_heading1_;
    weight_dist_ = weight_dist1_;
    dt_ = 1.0 / hz_;
    predict_time_ = predict_time1_;

    // ###### tf_buffer_とtf_listenerを初期化 ######
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ####### Subscriber #######
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/local_goal", 10, std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/obs_poses", 10, std::bind(&DWAPlanner::obs_poses_callback, this, std::placeholders::_1));

    // ###### Publisher ######
    // 【重要】Twist型に変更し、トピックを /cmd_vel に設定
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);

    flag_obs_poses_ = true;
}

// local_goalのコールバック関数
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    try
    {
        auto transform = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*msg, local_goal_, transform);
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        flag_local_goal_ = false;
    }
}

void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

int DWAPlanner::get_freq() { return hz_; }

void DWAPlanner::process()
{
    if (!flag_local_goal_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 2000, "Waiting for local_goal...");
    }
    if (!flag_obs_poses_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 2000, "Waiting for obs_poses...");
    }

    if (can_move()) {
        std::vector<double> input = calc_final_input();
        // 選ばれた速度をログ出力
        RCLCPP_INFO(this->get_logger(), "Best Input: v=%f, w=%f", input[0], input[1]);
        send_velocity(input[0], input[1], input[2]);
    } else {
         RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 1000, "Cannot move: Flags not set or Goal reached");
        send_velocity(0.0, 0.0, 0.0);
    }
}

bool DWAPlanner::can_move()
{
    if (!flag_local_goal_) return false; 
    
    double dist = std::hypot(local_goal_.point.x, local_goal_.point.y);
    return dist > goal_tolerance_;
}

void DWAPlanner::send_velocity(double vx, double vy, double yawrate)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;   
    msg.angular.z = yawrate;
    velocity_pub_->publish(msg);
}

std::vector<double> DWAPlanner::calc_final_input()
{
    std::vector<double> best_input{0.0, 0.0};          
    double max_score = -1e6;                      
    std::vector<State> best_traj;

    change_mode();
    calc_dynamic_window();

     for (double vx = dw_.min_vx; vx <= dw_.max_vx; vx += v_reso_) {
        for (double vy = dw_.min_vy; vy <= dw_.max_vy; vy += vy_reso_) {
            for (double w = dw_.min_yawrate; w <= dw_.max_yawrate; w += yawrate_reso_) {
                auto traj = calc_traj(vx, vy, w);
                double score = calc_evaluation(traj);
                if (score > max_score) {
                    max_score = score;
                    best_input = {vx, vy, w};
                    best_traj = traj;
                }
            }
        }
    }

    robot_.vx = best_input[0];
    robot_.vy = best_input[1];
    robot_.yawrate = best_input[2];
    if (!best_traj.empty()) visualize_traj(best_traj, pub_optimal_path_);
    return best_input;
}

void DWAPlanner::change_mode()
{
    double dist = std::hypot(local_goal_.point.x, local_goal_.point.y);
    if (dist < 1.0) {
        max_vel_ = max_vel2_;
        max_vel_y_ = max_vel_y2_;
        max_yawrate_ = max_yawrate2_;
        predict_time_ = predict_time2_;
    } else {
        max_vel_ = max_vel1_;
        max_vel_y_ = max_vel_y1_;
        max_yawrate_ = max_yawrate1_;
        predict_time_ = predict_time1_;
    }
}

void DWAPlanner::calc_dynamic_window()
{
    dw_.max_vx = std::min(max_vel_, robot_.vx + max_accel_ * dt_);
    dw_.min_vx = std::max(min_vel_, robot_.vx - max_accel_ * dt_);
    dw_.max_vy = std::min(max_vel_y_, robot_.vy + max_accel_ * dt_);
    dw_.min_vy = std::max(-max_vel_y_, robot_.vy - max_accel_ * dt_);
    dw_.max_yawrate = std::min(max_yawrate_, robot_.yawrate + max_dyawrate_ * dt_);
    dw_.min_yawrate = std::max(-max_yawrate_, robot_.yawrate - max_dyawrate_ * dt_);
}

std::vector<State> DWAPlanner::calc_traj(double vx, double vy, double yawrate) {
    std::vector<State> traj;
    State s = {0.0, 0.0, 0.0, vx, vy, yawrate};
    for (double t = 0; t <= predict_time_; t += dt_) {
        move(s, vx, vy, yawrate);
        traj.push_back(s);
    }
    return traj;
}

void DWAPlanner::move(State& s, double vx, double vy, double yawrate) {
    s.yaw += yawrate * dt_;
    s.x += (vx * std::cos(s.yaw) - vy * std::sin(s.yaw)) * dt_;
    s.y += (vx * std::sin(s.yaw) + vy * std::cos(s.yaw)) * dt_;
}

double DWAPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


double DWAPlanner::calc_evaluation(const std::vector<State>& traj) {
    return weight_heading_ * calc_heading_eval(traj) +
           weight_dist_ * calc_dist_eval(traj) +
           weight_vel_ * calc_vel_eval(traj);
}

double DWAPlanner::calc_heading_eval(const std::vector<State>& traj) {
    State last = traj.back();
    double target_yaw = std::atan2(local_goal_.point.y - last.y, local_goal_.point.x - last.x);
    double error = std::abs(normalize_angle(target_yaw - last.yaw));
    return (M_PI - error) / M_PI;
}

double DWAPlanner::calc_dist_eval(const std::vector<State>& traj) {
    double min_dist = search_range_;
    for (const auto& s : traj) {
        for (const auto& obs : obs_poses_.poses) {
            double d = std::hypot(s.x - obs.position.x, s.y - obs.position.y);
            if (d < robot_radius_) return -1e6;
            min_dist = std::min(min_dist, d);
        }
    }
    return min_dist / search_range_;
}

double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    return std::abs(traj[0].vx) / max_vel1_;
}

void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "base_link";
    path.header.stamp = this->now();
    for (const auto& s : traj) {

        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "base_link";
        p.pose.position.x = s.x;
        p.pose.position.y = s.y;
        path.poses.push_back(p);
    }
    pub->publish(path);
}