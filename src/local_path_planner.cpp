#include "local_path_planner/local_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言と取得 ######
    this->declare_parameter("hz", 10);
    this->declare_parameter("max_vel1", 0.5);   // 平常時最高速度
    this->declare_parameter("max_vel2", 0.2);   // 減速時最高速度
    this->declare_parameter("max_yawrate1", 1.0);
    this->declare_parameter("max_yawrate2", 0.5);
    this->declare_parameter("max_accel", 1.0);
    this->declare_parameter("predict_time", 2.0);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("weight_heading", 0.1);
    this->declare_parameter("weight_dist", 0.8);
    this->declare_parameter("weight_vel", 0.1);

    hz_ = this->get_parameter("hz").as_int();
    max_vel1_ = this->get_parameter("max_vel1").as_double();
    max_vel2_ = this->get_parameter("max_vel2").as_double();
    max_yawrate1_ = this->get_parameter("max_yawrate1").as_double();
    max_yawrate2_ = this->get_parameter("max_yawrate2").as_double();
    max_accel_ = this->get_parameter("max_accel").as_double();
    predict_time_ = this->get_parameter("predict_time").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    weight_heading1_ = this->get_parameter("weight_heading").as_double();
    weight_dist1_ = this->get_parameter("weight_dist").as_double();
    weight_vel_ = this->get_parameter("weight_vel").as_double();

    // 初期モード設定
    max_vel_ = max_vel1_;
    max_yawrate_ = max_yawrate1_;
    weight_heading_ = weight_heading1_;
    weight_dist_ = weight_dist1_;
    dt_ = 1.0 / (double)hz_;

    // ###### tf_buffer_とtf_listenerを初期化 ######
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ####### Subscriber #######
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/local_goal", 10, std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/obs_poses", 10, std::bind(&DWAPlanner::obs_poses_callback, this, std::placeholders::_1));

    // ###### Publisher ######
    pub_cmd_speed_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", 10);
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);
    pub_predict_path_ = this->create_publisher<nav_msgs::msg::Path>("/predict_paths", 10);
}

// local_goalのコールバック関数
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        // マップ座標系からbase_link座標系への変換を取得
        transform = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
        // 取得した変換を用いて，local_goal_ をロボット座標系 (base_link) に変換
        tf2::doTransform(*msg, local_goal_, transform);
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        flag_local_goal_ = false;
    }
}

// obs_posesのコールバック関数
void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// hzを返す関数
int DWAPlanner::get_freq() { return hz_; }

// メインプロセス
void DWAPlanner::process()
{
    if (can_move()) {
        std::vector<double> input = calc_final_input();
        roomba_control(input[0], input[1]);
    } else {
        roomba_control(0.0, 0.0);
    }
}

// ゴールに着くまでTrueを返す
bool DWAPlanner::can_move()
{
    if (!flag_local_goal_ || !flag_obs_poses_) return false;

    // ゴールとの距離を計算 (base_link座標系なので目標地点のx, yがそのまま距離になる)
    double dist_to_goal = std::hypot(local_goal_.point.x, local_goal_.point.y);
    
    if (dist_to_goal < goal_tolerance_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock_, 2000, "Goal Reached!");
        return false;
    }
    return true;
}

// Roombaの制御入力を行う
void DWAPlanner::roomba_control(const double velocity, const double yawrate)
{
    roomba_500driver_meiji::msg::RoombaCtrl msg;
    msg.mode = 11; // 任意の制御モード（ドライバの仕様に合わせる）
    msg.cntl_vx = velocity;
    msg.cntl_vtheta = yawrate;
    pub_cmd_speed_->publish(msg);
}

// 最適な制御入力を計算
std::vector<double> DWAPlanner::calc_final_input()
{
    std::vector<double> input{0.0, 0.0};          
    double max_score = -1e6;                      
    std::vector<State> best_traj;

    change_mode();
    calc_dynamic_window();

    // 並進速度(v)と旋回速度(w)の全組み合わせを探索
    for (double v = dw_.min_vel; v <= dw_.max_vel; v += 0.05) {
        for (double w = dw_.min_yawrate; w <= dw_.max_yawrate; w += 0.1) {
            
            std::vector<State> traj = calc_traj(v, w);
            double score = calc_evaluation(traj);

            if (score > max_score) {
                max_score = score;
                input[0] = v;
                input[1] = w;
                best_traj = traj;
            }
        }
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // pathの可視化
    if (!best_traj.empty()) {
        visualize_traj(best_traj, pub_optimal_path_, this->now());
    }

    return input;
}

void DWAPlanner::change_mode()
{
    // 簡易的な減速ロジック：目標が近い場合や障害物が近い場合にmax_vel2_を使用する等
    double dist_to_goal = std::hypot(local_goal_.point.x, local_goal_.point.y);
    if (dist_to_goal < 1.0) {
        max_vel_ = max_vel2_;
        max_yawrate_ = max_yawrate2_;
    } else {
        max_vel_ = max_vel1_;
        max_yawrate_ = max_yawrate1_;
    }
}

void DWAPlanner::calc_dynamic_window()
{
    // 車両モデルによるWindow (Vs)
    double Vs_min_v = 0.0;
    double Vs_max_v = max_vel_;
    double Vs_min_w = -max_yawrate_;
    double Vs_max_w = max_yawrate_;

    // 運動モデルによるWindow (Vd)
    double Vd_min_v = roomba_.velocity - max_accel_ * dt_;
    double Vd_max_v = roomba_.velocity + max_accel_ * dt_;
    double Vd_min_w = roomba_.yawrate - max_accel_ * dt_; // 旋回も同様の加速度制限と仮定
    double Vd_max_w = roomba_.yawrate + max_accel_ * dt_;

    // 最終的なDynamic Window (交差部分)
    dw_.min_vel = std::max(Vs_min_v, Vd_min_v);
    dw_.max_vel = std::min(Vs_max_v, Vd_max_v);
    dw_.min_yawrate = std::max(Vs_min_w, Vd_min_w);
    dw_.max_yawrate = std::min(Vs_max_w, Vd_max_w);
}

std::vector<State> DWAPlanner::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> traj;
    State state = {0.0, 0.0, 0.0, velocity, yawrate}; // 初期位置は常にbase_linkの原点
    for (double t = 0; t <= predict_time_; t += dt_) {
        move(state, velocity, yawrate);
        traj.push_back(state);
    }
    return traj;
}

void DWAPlanner::move(State& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate * dt_;
    state.x += velocity * std::cos(state.yaw) * dt_;
    state.y += velocity * std::sin(state.yaw) * dt_;
}

double DWAPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double DWAPlanner::calc_evaluation(const std::vector<State>& traj)
{
    double heading_score  = weight_heading_ * calc_heading_eval(traj);
    double distance_score = weight_dist_    * calc_dist_eval(traj);
    double velocity_score = weight_vel_     * calc_vel_eval(traj);

    return heading_score + distance_score + velocity_score;
}

double DWAPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    // 軌跡の終端での目標方向への向きやすさ
    State last = traj.back();
    double dx = local_goal_.point.x - last.x;
    double dy = local_goal_.point.y - last.y;
    double target_yaw = std::atan2(dy, dx);
    double error_yaw = normalize_angle(target_yaw - last.yaw);
    
    return (M_PI - std::abs(error_yaw)) / M_PI; // 0~1に正規化
}

double DWAPlanner::calc_dist_eval(const std::vector<State>& traj)
{
    double min_dist = 1e6;
    for (const auto& s : traj) {
        for (const auto& obs : obs_poses_.poses) {
            double dist = std::hypot(s.x - obs.position.x, s.y - obs.position.y);
            if (dist < 0.3) return -1e6; // 衝突コスト（非常に低いスコア）
            min_dist = std::min(min_dist, dist);
        }
    }
    return min_dist;
}

double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    return traj.front().velocity / max_vel_; // 速いほど高スコア
}

void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, rclcpp::Time now)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "base_link";
    path.header.stamp = now;
    for (const auto& s : traj) {
        geometry_msgs::msg::PoseStamped p;
        p.pose.position.x = s.x;
        p.pose.position.y = s.y;
        path.poses.push_back(p);
    }
    pub->publish(path);
}