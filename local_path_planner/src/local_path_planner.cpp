#include "local_path_planner/local_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言と取得 ######
    this->declare_parameter("hz", 10);
    this->declare_parameter("max_vel1", 0.5); //0.5
    this->declare_parameter("max_vel_y1", 0.3);//0.3
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
    this->declare_parameter("weight_heading1", 0.5);
    this->declare_parameter("weight_dist1", 0.5);
    this->declare_parameter("weight_vel", 0.3);
    this->declare_parameter("roomba_radius", 0.15);
    this->declare_parameter("radius_margin1", 0.2);
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
    radius_margin1_ = this->get_parameter("radius_margin1").as_double();
    search_range_ = this->get_parameter("search_range").as_double();
    max_yawrate1_ = this->get_parameter("max_yawrate1").as_double();
    max_yawrate2_ = this->get_parameter("max_yawrate2").as_double();
    predict_time1_ = this->get_parameter("predict_time1").as_double();
    predict_time2_ = this->get_parameter("predict_time2").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    weight_heading1_ = this->get_parameter("weight_heading1").as_double();
    weight_dist1_ = this->get_parameter("weight_dist1").as_double();
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
        "/obstacle_points", 10, std::bind(&DWAPlanner::obs_poses_callback, this, std::placeholders::_1));

    // ###### Publisher ######
    // 【重要】Twist型に変更し、トピックを /cmd_vel に設定
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);
}

// local_goalのコールバック関数
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_msg_ = *msg;
    flag_local_goal_ = true;
}

void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

int DWAPlanner::get_freq() { return hz_; }

void DWAPlanner::process()
{
    if (!flag_obs_poses_ || obs_poses_.header.frame_id.empty()) return;
    // 修正：frame_id が空なら変換処理そのものをスキップする
    try {
        // goal_msg_ など、受信した元のメッセージをメンバ変数に保存しておく必要があります
        auto transform = tf_buffer_->lookupTransform("base_link", goal_msg_.header.frame_id, tf2::TimePointZero);
        // local_goal_original_ は map座標系の PointStamped
        tf2::doTransform(goal_msg_, local_goal_, transform); 
    } catch (tf2::TransformException &ex) {
        return; // TFが引けないときは処理しない
    }
    geometry_msgs::msg::PoseArray transformed_obs;
// process() 内で、障害物も base_link に変換する
    if (flag_obs_poses_) {
        try {
            // 障害物データの frame_id から base_link への変換を取得
            auto trans_obs = tf_buffer_->lookupTransform("base_link", obs_poses_.header.frame_id, tf2::TimePointZero);
            // ######追加
            // auto trans_obs = tf_buffer_->lookupTransform("base_link", obs_poses_.header.frame_id, obs_poses_.header.stamp, tf2::durationFromSec(0.1));
            transformed_obs.poses.clear();
            // ######
            for (auto& p_in : obs_poses_.poses) {
                geometry_msgs::msg::Pose p_out;
                // 座標変換の適用（簡易的な実装例）
                tf2::doTransform(p_in, p_out, trans_obs);
                transformed_obs.poses.push_back(p_out);
            }
            // obs_poses_ = transformed_obs; // 変換後の座標で上書き
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error (obs): %s", ex.what());
            return;
        }
    }

    if (can_move()) {
        change_mode();
        std::vector<double> input = calc_final_input(transformed_obs);
        
        // もし全経路が衝突判定なら強制停止
        if (input[0] == 0.0 && std::abs(input[2]) < 0.01) {
            RCLCPP_WARN(this->get_logger(), "No Safe Path!");
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 500, "Best Input: vx=%f, vy=%f, w=%f", input[0], input[1], input[2]);
        send_velocity(input[0], input[1], input[2]);
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 2000, "Goal Reached. Stopping...");
        send_velocity(0.0, 0.0, 0.0);
    }
}

bool DWAPlanner::can_move()
{
    if(!flag_local_goal_){
        RCLCPP_INFO(this->get_logger(), "not local goal!.");
        return false;
    }
    if(!flag_obs_poses_){
        RCLCPP_INFO(this->get_logger(), "not obs pose!.");
        return false;
    }
    if (!flag_local_goal_ || !flag_obs_poses_){
        RCLCPP_INFO(this->get_logger(), "Cannot move!.");
        return false;
    }
    double dist = std::hypot(local_goal_.point.x, local_goal_.point.y);

    // ##### 修正箇所(追加) #####
    if (local_goal_.point.x < 0 && dist < 0.5) {
        RCLCPP_INFO(this->get_logger(), "Goal behind robot. Stopping.");
        return false;
    }
    
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

std::vector<double> DWAPlanner::calc_final_input(const geometry_msgs::msg::PoseArray& current_obs)
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
                double score = calc_evaluation(traj, current_obs);

                // if (vx > 0.1 && std::abs(w) < 0.01 && std::abs(vy) < 0.01) {
                //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                //         "--- Straight Candidate --- vx: %f, score: %f", vx, score);
                // }

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
    s.x += (vx * std::cos(s.yaw) - vy * std::sin(s.yaw)) * dt_;
    s.y += (vx * std::sin(s.yaw) + vy * std::cos(s.yaw)) * dt_;
    s.yaw += yawrate * dt_;
}

double DWAPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


double DWAPlanner::calc_evaluation(const std::vector<State>& traj, const geometry_msgs::msg::PoseArray& current_obs) {
    return weight_heading_ * calc_heading_eval(traj) +
           weight_dist_ * calc_dist_eval(traj, current_obs) +
           weight_vel_ * calc_vel_eval(traj);
}

double DWAPlanner::calc_heading_eval(const std::vector<State>& traj) {
    State last = traj.back();
    // double target_yaw = std::atan2(local_goal_.point.y - last.y, local_goal_.point.x - last.x);
    // double error = std::abs(normalize_angle(target_yaw - last.yaw));
    
    //  ##### 修正箇所 #####
    double dist = std::hypot(local_goal_.point.x, local_goal_.point.y);
    if (dist < 0.5) return 1.0;
    double target_angle = std::atan2(local_goal_.point.y, local_goal_.point.x);
    double error = std::abs(normalize_angle(target_angle - last.yaw));
    return (M_PI - error) / M_PI;
    // ###################
}

double DWAPlanner::calc_dist_eval(const std::vector<State>& traj, const geometry_msgs::msg::PoseArray& obs_list) {
    double min_dist = search_range_;
    for (const auto& s : traj) {
        for (const auto& obs : obs_list.poses) {
            double d1 = std::hypot(s.x - obs.position.x, s.y - obs.position.y);
            // double d = std::hypot(obs.position.x, obs.position.y);
            // if (d < 0.1) continue;
            if (d1 < robot_radius_ + radius_margin1_) return -1e6;//-1e6
            min_dist = std::min(min_dist, d1);
        }
    }
    double score_dist = min_dist / search_range_;
    // return score_dist * score_dist;
    return score_dist;
}

double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    // if (traj[0].vx < 0) return 0.0; 

    double vx_score = traj[0].vx / max_vel1_;
    // double vx_score = std::abs(traj[0].vx) / max_vel1_;
    double w_score = std::abs(traj[0].yawrate) / max_yawrate_;
    // 【強力な修正】横速度 vy が少しでもあればスコアを大幅に減点する
    double vy_penalty = std::abs(traj[0].vy) / max_vel_y1_;
    return (vx_score * 0.7) - (vy_penalty * 0.3);

    // return traj[0].vx / max_vel1_;

    // return std::abs(traj[0].vx) / max_vel1_;
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