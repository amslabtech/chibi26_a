/*
DWA: Dynamic Window Approach

速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner/local_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    this->declare_parameter("is_visible",true);
    // 制御周波数 [Hz]
    this->declare_parameter("hz",10);
    // frame_id
    this->declare_parameter("robot_frame","base_link");
    // 速度制限
    this->declare_parameter("max_vel1",0.5); //[m/s]（平常時）
    this->declare_parameter("max_vel2",0.3); //[m/s]（減速時）
    this->declare_parameter("avoid_thres_vel",0.3); //[m/s]（回避中か判断する閾値）
    this->declare_parameter("min_vel",0.0); //[m/s]
    this->declare_parameter("max_yawrate1",1.5); //[rad/s]（平常時）
    this->declare_parameter("max_yawrate2",1.0); //[rad/s]（減速時）
    this->declare_parameter("turn_thres_yawrate",1.0); //[rad/s]（旋回中か判断する閾値）
    this->declare_parameter("mode_log_time",10);
    // 加速度制限
    this->declare_parameter("max_accel",1.0); //[m/s^2]
    this->declare_parameter("max_dyawrate",2.0); //[rad/s^2]
    // 速度解像度
    this->declare_parameter("vel_reso",0.03); //[m/s]
    this->declare_parameter("yawrate_reso",0.05); //[rad/s]
    // 停止状態か判断する閾値
    this->declare_parameter("stop_vel_th",0.01);
    this->declare_parameter("stop_yawrate_th",0.05);
    // 時間 [s]
    this->declare_parameter("dt",0.1);
    this->declare_parameter("predict_time1",2.0);
    this->declare_parameter("predict_time2",3.0);
    // 機体サイズ(半径) [m]
    this->declare_parameter("roomba_radius",0.17);
    this->declare_parameter("radius_margin1",0.05); //（平常時）
    this->declare_parameter("radius_margin2",0.1); //（減速時）
    // 重み定数 [-]
    this->declare_parameter("weight_heading1",2.0); //（平常時）
    this->declare_parameter("weight_heading2",1.0); //（減速時）
    this->declare_parameter("weight_dist1",0.3); //（平常時）
    this->declare_parameter("weight_dist2",0.1); //（減速時）
    this->declare_parameter("weight_vel",0.1);
    // 許容誤差 [m]
    this->declare_parameter("goal_tolerance",0.2);
    // 評価関数distで探索する範囲 [m]
    this->declare_parameter("search_range",3.0);

    // ###### パラメータの取得 ######
    is_visible_ = this->get_parameter("is_visible").as_bool();

    hz_ = this->get_parameter("hz").as_int();

    max_vel1_ = this->get_parameter("max_vel1").as_double();
    max_vel2_ = this->get_parameter("max_vel2").as_double();
    avoid_thres_vel_ = this->get_parameter("avoid_thres_vel").as_double();
    min_vel_ = this->get_parameter("min_vel").as_double();
    max_yawrate1_ = this->get_parameter("max_yawrate1").as_double();
    max_yawrate2_ = this->get_parameter("max_yawrate2").as_double();
    turn_thres_yawrate_ = this->get_parameter("turn_thres_yawrate").as_double();
    mode_log_time_ = this->get_parameter("mode_log_time").as_double();

    max_accel_ = this->get_parameter("max_accel").as_double();
    max_dyawrate_ = this->get_parameter("max_dyawrate").as_double();

    vel_reso_ = this->get_parameter("vel_reso").as_double();
    yawrate_reso_ = this->get_parameter("yawrate_reso").as_double();

    stop_vel_th_ = this->get_parameter("stop_vel_th").as_double();
    stop_yawrate_th_ = this->get_parameter("stop_yawrate_th").as_double();

    dt_ = this->get_parameter("dt").as_double();
    predict_time1_ = this->get_parameter("predict_time1").as_double();
    predict_time2_ = this->get_parameter("predict_time2").as_double();

    roomba_radius_ = this->get_parameter("roomba_radius").as_double();
    radius_margin1_ = this->get_parameter("radius_margin1").as_double();
    radius_margin2_ = this->get_parameter("radius_margin2").as_double();

    weight_heading1_ = this->get_parameter("weight_heading1").as_double();
    weight_heading2_ = this->get_parameter("weight_heading2").as_double();
    weight_dist1_ = this->get_parameter("weight_dist1").as_double();
    weight_dist2_ = this->get_parameter("weight_dist2").as_double();
    weight_vel_ = this->get_parameter("weight_vel").as_double();

    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    search_range_ = this->get_parameter("search_range").as_double();

    // ###### tf_buffer_とtf_listenerを初期化 ######
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ####### Subscriber #######
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>("local_goal" ,rclcpp::QoS(1), std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("obs" ,rclcpp::QoS(1), std::bind(&DWAPlanner::obs_poses_callback, this, std::placeholders::_1));

    // ###### Publisher ######
    pub_cmd_speed_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("roomba_control", 10);
    if (is_visible_) {
        pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("optimal_path", 10);
        pub_predict_path_ = this->create_publisher<nav_msgs::msg::Path>("predict_paths", 10);
    }
}

// local_goalのコールバック関数
// local_goalはマップ座標系(map)だが，実際の移動に合わせるためにルンバ座標系(base_link)に変換する処理を行う
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        // tfを取得，成功したらフラグを立てる
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(robot_frame_, msg->header.frame_id, tf2::TimePointZero);

        // 取得した変換を用いて、PointStampedを直接変換
        tf2::doTransform(*msg, local_goal_, transform);

        flag_local_goal_ = true;

    }
    catch(tf2::TransformException& ex)
    {
        // 取得に失敗した場合，警告を出力しフラグをリセット
        RCLCPP_WARN(this->get_logger(), "Could not transform local_goal: %s", ex.what());
        flag_local_goal_ = false;

    }
    // 取得した変換を用いて，local_goal_ をロボット座標系 (base_link) に変換


}

// obs_posesのコールバック関数
void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// hzを返す関数
int DWAPlanner::get_freq()
{
    return hz_;
}

// 唯一，main文で実行する関数
// can_move()がTrueのとき，calc_final_input()を実行し，速度と旋回速度を計算
// それ以外の場合は，速度と旋回速度を0にする
void DWAPlanner::process()
{
    if(can_move() && flag_local_goal_){
        calc_dynamic_window();

        std::vector<double> input = calc_final_input();

        roomba_control(input[0] ,input[1]);
    }else{
        roomba_control(0.0 ,0.0);
    }
}

// ゴールに着くまでTrueを返す
bool DWAPlanner::can_move()
{
    if(!flag_local_goal_) return false;

    // ロボット中心(0,0)とlocal_goal_の距離を計算
    double dist_to_goal = std::hypot(local_goal_.point.x, local_goal_.point.y);

    // 許容誤差以内なら停止(false), 遠ければ続行(true)
    return dist_to_goal > goal_tolerance_;
}

// Roombaの制御入力を行う
void DWAPlanner::roomba_control(const double velocity, const double yawrate)
{
    roomba_500driver_meiji::msg::RoombaCtrl msg;
    msg.mode = 11; // 任意のモード（ドライバの仕様に合わせる）
    msg.cntl.linear.x = velocity;
    msg.cntl.angular.z = yawrate;
    pub_cmd_speed_->publish(msg);
}

// 最適な制御入力を計算
std::vector<double> DWAPlanner::calc_final_input()
{
    // 変数を定義，初期化
    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 旋回状況に応じた減速機能
    change_mode();

    // ダイナミックウィンドウを計算
    calc_dynamic_window();

    int i = 0; // 現在の軌跡のインデックス保持用
    // ###### 並進速度と旋回速度のすべての組み合わせを評価 ######
    for(double v = dw_.min_vel; v <= dw_.max_vel; v += vel_reso_){
        for(double w = dw_.min_yawrate; w <= dw_.max_yawrate; w += yawrate_reso_){
            std::vector<State> traj = calc_traj(v ,w);

            double score = calc_evaluation(traj);

            if(is_visible_) trajectories.push_back(traj);

            if(score > max_score){
                max_score = score;
                input[0] = v;
                input[1] = w;
                index_of_max_score = i;
            }
            i++;
        }
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // ###### pathの可視化 #######
    if(is_visible_ && !trajectories.empty()){
        visualize_traj(trajectories[index_of_max_score] ,pub_optimal_path_ ,this->now());
    }

    return input;
}

// 旋回状況に応じた減速機能
// ロボットの旋回速度や速度によって減速モードに切り替える（普段よりも遅く動く）
void DWAPlanner::change_mode()
{
    if(std::abs(roomba_.yawrate) > turn_thres_yawrate_){
        max_vel_ = max_vel2_;
        weight_dist_ = weight_dist2_;
        weight_heading_ = weight_heading2_;
        predict_time_ = predict_time2_;
    }else{
        max_vel_ = max_vel1_;
        weight_dist_ = weight_dist1_;
        weight_heading_ = weight_heading1_;
        predict_time_ = predict_time1_;
    }
}

// Dynamic Windowを計算
void DWAPlanner::calc_dynamic_window()
{
    // ###### 車両モデルによるWindow ######
    double Vs[] = {min_vel_ ,max_vel_, -max_yawrate1_, max_yawrate1_};

    // ####### 運動モデルによるWindow #######
    double Vd[] = {
        roomba_.velocity - max_accel_ * dt_,
        roomba_.velocity + max_accel_ * dt_,
        roomba_.yawrate - max_dyawrate_ * dt_,
        roomba_.yawrate + max_dyawrate_ * dt_
    };

    // ###### 最終的なDynamic Window ######
    dw_.min_vel = std::max(Vs[0], Vd[0]);
    dw_.max_vel = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 指定された予測時間までロボットの状態を更新し，予測軌跡を生成
std::vector<State> DWAPlanner::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> traj;
    State temp_state = {0.0, 0.0, 0.0, velocity, yawrate};// ロボット座標系なので初期位置は(0,0,0)

    for(double t = 0; t <= predict_time_; t += dt_){
        move(temp_state, velocity, yawrate);
        traj.push_back(temp_state);
    }
    return traj;
}

// 予測軌跡作成時における仮想ロボットを移動
void DWAPlanner::move(State& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate * dt_;
    state.yaw = normalize_angle(state.yaw);
    state.x += velocity * std::cos(state.yaw) * dt_;
    state.y += velocity * std::sin(state.yaw) * dt_;
    state.velocity = velocity;
    state.yawrate = yawrate;
}

// angleを適切な角度(-M_PI ~ M_PI)の範囲にして返す
double DWAPlanner::normalize_angle(double angle)
{
    while(angle > M_PI) angle -= 2.0 * M_PI;
    while(angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 評価関数を計算
double DWAPlanner::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = weight_heading_ * calc_heading_eval(traj);
    const double distance_score = weight_dist_    * calc_dist_eval(traj);
    const double velocity_score = weight_vel_     * calc_vel_eval(traj);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
// 軌跡のゴール方向への向きやすさを評価する関数
double DWAPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    double last_x = traj.back().x;
    double last_y = traj.back().y;
    double last_yaw = traj.back().yaw;

    // 目標地点への角度
    double angle_to_goal = std::atan2(local_goal_.point.y - last_y, local_goal_.point.x - last_x);
    double target_angle = angle_to_goal - last_yaw;

    // 誤差を -PI ~ PI に正規化し、その絶対値が小さい（正面を向いている）ほど高評価
    return (M_PI - std::abs(normalize_angle(target_angle))) / M_PI;
}

// distの評価関数を計算
// 軌跡の障害物回避性能を評価する関数
double DWAPlanner::calc_dist_eval(const std::vector<State>& traj)
{
    double min_dist = search_range_;

    for(const auto& state : traj){
        for(const auto& obs : obs_poses_.poses){
            double dist = std::hypot(state.x - obs.position.x, state.y - obs.position.y);

            // 衝突判定：Roombaの半径＋マージン以下なら最悪評価
            if(dist <= roomba_radius_) return -1e6;

            if(dist <= min_dist) min_dist = dist;
        }
    }
    return min_dist / search_range_;
}

// velocityの評価関数を計算
// 軌跡の速度評価を計算する関数
double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    return traj.front().velocity / max_vel_;
}

// 軌跡を可視化するための関数
void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = robot_frame_;
    path.header.stamp = now;

    for(const auto& state : traj){
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        path.poses.push_back(pose);
    }
    pub_local_path->publish(path);
}