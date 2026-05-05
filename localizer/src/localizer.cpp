#include "localizer/localizer.hpp"

// コンストラクタ
Localizer::Localizer() : Node("team_localizer")
{ 
    // ----- パラメータの宣言と取得 -----

    // 1. 基本設定
    hz_ = this->declare_parameter("hz", 10);
    max_particle_num_ = this->declare_parameter("max_particle_num", 500);
    min_particle_num_ = this->declare_parameter("min_particle_num", 100);
    move_dist_th_ = this->declare_parameter("move_dist_th", 0.05);
    move_angle_th_ = this->declare_parameter("move_angle_th", 0.05);

    // 2. 初期位置、分散
    init_x_ = this->declare_parameter("init_x", 0.0);
    init_y_ = this->declare_parameter("init_y", 0.0);
    init_yaw_ = this->declare_parameter("init_yaw", 0.0);
    init_x_dev_ = this->declare_parameter("init_x_dev", 0.1);
    init_y_dev_ = this->declare_parameter("init_y_dev", 0.1);
    init_yaw_dev_ = this->declare_parameter("init_yaw_dev", 0.05);

    // 3. リセット関連
    alpha_th_ = this->declare_parameter("alpha_th", 0.01);
    expansion_threshold_ = this->declare_parameter("expansion_threshold", 0.005);
    reset_count_limit_ = this->declare_parameter("reset_count_limit", 5);
    expansion_x_dev_ = this->declare_parameter("expansion_x_dev", 0.5);
    expansion_y_dev_ = this->declare_parameter("expansion_y_dev", 0.5);
    expansion_yaw_dev_ = this->declare_parameter("expansion_yaw_dev", 0.2);

    // 4. センサ関連
    laser_step_ = this->declare_parameter<int>("laser_step", 10);
    sensor_noise_ratio_ = this->declare_parameter<double>("sensor_noise_ratio", 0.05);
    ignore_angle_range_list_ = this->declare_parameter<std::vector<double>>("ignore_angle_range_list", std::vector<double>{});

    // 5. OdomModel関連 (ff, fr, rf, rr)
    ff_ = this->declare_parameter<double>("ff", 0.17);
    fr_ = this->declare_parameter<double>("fr", 0.0005);
    rf_ = this->declare_parameter<double>("rf", 0.13);
    rr_ = this->declare_parameter<double>("rr", 0.2);

    // 6. その他のフラグ
    flag_init_noise_ = this->declare_parameter<bool>("flag_init_noise", true);
    flag_reverse_ = this->declare_parameter<bool>("flag_reverse", false);

    // ----- オブジェクトの初期化 -----
    // odometryのモデルの初期化
    odom_model_ = OdomModel(ff_, fr_, rf_, rr_);

    // Subscriberの設定
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), 
        std::bind(&Localizer::map_callback, this, std::placeholders::_1));
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Localizer::odom_callback, this, std::placeholders::_1));
    
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Localizer::laser_callback, this, std::placeholders::_1));

    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::SystemDefaultsQoS(), 
        std::bind(&Localizer::callback_initialpose, this, std::placeholders::_1));

    // Publisherの設定
    pub_estimated_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);
    pub_particle_cloud_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);

    // フレームIDの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";

    // メモリ確保と初期化
    particle_cloud_msg_.poses.reserve(max_particle_num_);
    
    // 変数の初期化
    flag_broadcast_ = true;
    is_visible_ = true;
    reset_counter = 0;

    // パーティクルの初期配置
    initialize_particles(init_x_, init_y_, init_yaw_);

    RCLCPP_INFO(this->get_logger(), "Localizer: Initialized with all parameters.");
}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    flag_map_ = true;
    RCLCPP_INFO(this->get_logger(), "Map received.");

}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    prev_odom_ = *msg; 
    flag_odom_ = true;
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_laser_ = true;
}

// hz_を返す関数
int Localizer::getOdomFreq()
{
    return hz_;
}

// ロボットとパーティクルの推定位置の初期化
void Localizer::initialize()
{
    // 推定位置の初期化
    double i_x, i_y, i_yaw;
    this->get_parameter("init_x", i_x);
    this->get_parameter("init_y", i_y);
    this->get_parameter("init_yaw", i_yaw);

    // 初期位置近傍にパーティクルを配置
    for (int i = 0; i < max_particle_num_; i++) {
        // 初期位置の周囲に正規分布で散らす
        double px = norm_rv(i_x, 0.2);
        double py = norm_rv(i_y, 0.2);
        double pyaw = norm_rv(i_yaw, 0.1);
        particles_.emplace_back(px, py, pyaw, 1.0 / max_particle_num_);
    }
    
    estimated_pose_.set(i_x, i_y, i_yaw);
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    
    if (flag_map_ && flag_laser_ && flag_odom_) 
    {
        // 1. 移動更新
        motion_update(); 

        // 2. 観測更新 (内部で scan と map を使用)
        localize(); 

        // 3. 結果の配信
        broadcast_odom_state(); 
        publish_particles(); 
        publish_estimated_pose();

        // odomとscanは次回の更新のためにフラグを下ろす
        // mapは一度受け取れば使い回すので true のままでOK
        flag_odom_ = false;
        flag_laser_ = false; 
    }

}

// 適切な角度(-M_PI ~ M_PI)を返す
double Localizer::normalize_angle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ランダム変数生成関数（正規分布）
double Localizer::norm_rv(const double mean, const double stddev)
{
    //static std::mt19937 engine(std::random_device{}());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(engine_);
}

// パーティクルの重みの初期化
void Localizer::reset_weight()
{
    for (auto& p : particles_) {
        p.set_weight(1.0 / particles_.size());
    }
}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
// map座標系からみたbase_link座標系の位置と姿勢，odom座標系からみたbase_link座標系の位置と姿勢から計算
void Localizer::broadcast_odom_state()
{
    if(flag_broadcast_)
    {
        // TF Broadcasterの実体化
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        // broadcasterの初期化
        odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系からみたbase_link座標系の位置と姿勢の取得
        tf2::Transform map_to_base;
        map_to_base.setOrigin(tf2::Vector3(estimated_pose_.x(), estimated_pose_.y(), 0.0));
        tf2::Quaternion q;
        q.setRPY(0, 0, estimated_pose_.yaw());
        map_to_base.setRotation(q);

        // odom座標系からみたbase_link座標系の位置と姿勢の取得
        tf2::Transform odom_to_base;
        tf2::fromMsg(last_odom_.pose.pose, odom_to_base);
        // map座標系からみたodom座標系の位置と姿勢を計算（回転行列を使った単純な座標変換）
        tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();
        // yawからquaternionを作成
        tf2::Quaternion map_to_odom_quat;

        // odom座標系odomの位置姿勢情報格納するための変数
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納
        odom_state.header.stamp = this->now();

        // 親フレーム・子フレームの指定
        odom_state.header.frame_id = "map";
        odom_state.child_frame_id  = "odom";

        // map座標系からみたodom座標系の原点位置と方向の格納
        odom_state.transform = tf2::toMsg(map_to_odom);

        // tf情報をbroadcast(座標系の設定)
        odom_state_broadcaster->sendTransform(odom_state);
    }

}

// 自己位置推定
// 動作更新と観測更新を行う
void Localizer::localize()
{
    observation_update();
}

// 動作更新
// ロボットの微小移動量を計算し，パーティクルの位置をノイズを加えて更新
void Localizer::motion_update()
{
    // 1. 差分の計算
    double curr_yaw = get_yaw_from_quat(prev_odom_.pose.pose.orientation);
    double last_yaw = get_yaw_from_quat(last_odom_.pose.pose.orientation);

    double dx = prev_odom_.pose.pose.position.x - last_odom_.pose.pose.position.x;
    double dy = prev_odom_.pose.pose.position.y - last_odom_.pose.pose.position.y;
    double dth = normalize_angle(curr_yaw - last_yaw);
    double dist = std::sqrt(dx * dx + dy * dy);

    // 2. 閾値チェック
    if (dist > move_dist_th_ || std::abs(dth) > move_angle_th_) 
    {
        odom_model_.set_dev(dist, dth);

        for (auto& p : particles_) 
        {
            // パーティクル自身の向き(p.pose_.yaw())を基準にするのが重要
            double relative_direction = normalize_angle(std::atan2(dy, dx) - last_yaw);
            
            p.pose_.move(dist, relative_direction, dth, 
                         odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
        }

        // 3. 更新の確定：ここで last_odom_ を更新する
        last_odom_ = prev_odom_;
    }
}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    double total_weight = 0.0;

    for (auto& p : particles_) {
        double w = p.likelihood(
            map_,
            laser_,
            sensor_noise_ratio_,
            laser_step_,
            ignore_angle_range_list_
        );
        p.set_weight(w);
        total_weight += w;
    }

    // 正規化前の平均尤度
    double alpha = total_weight / static_cast<double>(particles_.size());

    if (total_weight > 0.0) {
        for (auto& p : particles_) {
            p.set_weight(p.weight() / total_weight);
        }
    } else {
        reset_weight();
    }

    estimate_pose();

    expansion_resetting(alpha);

    resampling(alpha);
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{
    if (particles_.empty()) {
        return 0.0;
    }

    double sum_likelihood = 0.0;

    // 全パーティクルの現在の重み（尤度）を合計する
    for (const auto& p : particles_) {
        sum_likelihood += p.weight();
    }

    // 平均尤度（周辺尤度）を返す
    return sum_likelihood / static_cast<double>(particles_.size());
}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
void Localizer::estimate_pose()
{
    // 加重平均による推定
    double mean_x = 0, mean_y = 0, sum_sin = 0, sum_cos = 0;
    for (const auto& p : particles_) {
        double w = p.weight();
        mean_x += p.pose_.x() * w;
        mean_y += p.pose_.y() * w;
        sum_sin += std::sin(p.pose_.yaw()) * w;
        sum_cos += std::cos(p.pose_.yaw()) * w;
    }
    estimated_pose_.set(mean_x, mean_y, std::atan2(sum_sin, sum_cos));
}

// 重みの正規化
void Localizer::normalize_belief()
{
    double total_weight = 0.0;
    for (const auto& p : particles_) {
        total_weight += p.weight();
    }

    // 合計が極端に小さい（全滅に近い）場合の安全処理
    if (total_weight < 1e-9) {
        reset_weight(); // 均一な重みに戻す
        return;
    }

    // 各重みを合計値で割る
    for (auto& p : particles_) {
        p.set_weight(p.weight() / total_weight);
    }
}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting(double alpha)
{
    if (alpha < expansion_threshold_) {
        reset_counter++;
    } else {
        reset_counter = 0;
        return;
    }

    if (reset_counter < reset_count_limit_) {
        return;
    }

    emcl_reset();
    reset_counter = 0;
}

void Localizer::emcl_reset()
{
    int n = particles_.size();
    int local_num = static_cast<int>(n * 0.9);

    // 70%：推定位置周辺に広げる
    for (int i = 0; i < local_num; i++) {
        double px = norm_rv(estimated_pose_.x(), expansion_x_dev_);
        double py = norm_rv(estimated_pose_.y(), expansion_y_dev_);
        double pyaw = norm_rv(estimated_pose_.yaw(), expansion_yaw_dev_);
        particles_[i].pose_.set(px, py, pyaw);
    }

    // 30%：地図全体にランダム配置
    for (int i = local_num; i < n; i++) {
        set_random_particle_in_free_space(particles_[i]);
    }

    reset_weight();
}

void Localizer::set_random_particle_in_free_space(Particle& p)
{
    if (map_.data.empty()) {
        return;
    }

    std::uniform_int_distribution<int> x_dist(0, map_.info.width - 1);
    std::uniform_int_distribution<int> y_dist(0, map_.info.height - 1);
    std::uniform_real_distribution<double> yaw_dist(-M_PI, M_PI);

    // 無限ループ防止
    const int max_trial = 1000;

    for (int trial = 0; trial < max_trial; trial++) {
        int grid_x = x_dist(engine_);
        int grid_y = y_dist(engine_);

        int index = grid_y * map_.info.width + grid_x;

        // 空きセルだけ採用
        if (map_.data[index] == 0) {
            double x = map_.info.origin.position.x
                     + (grid_x + 0.5) * map_.info.resolution;

            double y = map_.info.origin.position.y
                     + (grid_y + 0.5) * map_.info.resolution;

            double yaw = yaw_dist(engine_);

            p.pose_.set(x, y, yaw);
            return;
        }
    }

    // 空きセルが見つからなかった場合の保険
    double px = norm_rv(estimated_pose_.x(), expansion_x_dev_);
    double py = norm_rv(estimated_pose_.y(), expansion_y_dev_);
    double pyaw = norm_rv(estimated_pose_.yaw(), expansion_yaw_dev_);

    p.pose_.set(px, py, pyaw);
}

// リサンプリング（系統サンプリング）
// 周辺尤度に応じてパーティクルをリサンプリング
void Localizer::resampling(const double alpha)
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;
    double sum = 0.0;
    for (const auto& p : particles_) {
        sum += p.weight();
        accum.push_back(sum);
    }

    // サンプリングのスタート位置とステップを設定
    std::vector<Particle> old(particles_);
    int size = particles_.size();
    particles_.clear();

    // particle数の動的変更
    double step = 1.0 / size;
    double r = (double)rand() / RAND_MAX * step;

    for (int j = 0; j < size; j++) {
        double U = r + j * step;
        auto it = std::lower_bound(accum.begin(), accum.end(), U);
        int index = std::distance(accum.begin(), it);
        particles_.push_back(old[index]);
    }
    // 重みを初期化
    reset_weight(); // リサンプリング後は重みを均一化
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{
    estimated_pose_msg_.header.stamp = this->now();
    estimated_pose_msg_.pose.position.x = estimated_pose_.x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.y();
    
    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose_.yaw());
    estimated_pose_msg_.pose.orientation = tf2::toMsg(q);

    pub_estimated_pose_->publish(estimated_pose_msg_);
}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    if(is_visible_ && !particles_.empty() )
    {
        // 1. ヘッダー更新
        particle_cloud_msg_.header.stamp = this->now();
        particle_cloud_msg_.header.frame_id = map_.header.frame_id;

        // 2. サイズ調整
        if (particle_cloud_msg_.poses.size() != particles_.size()) {
            particle_cloud_msg_.poses.resize(particles_.size());
        }

        // 3. 各パーティクルの情報をメッセージ形式に変換
        for (size_t i = 0; i < particles_.size(); ++i) {
            const auto& p_pose = particles_[i].pose_;

            // 位置 (x, y) の代入
            particle_cloud_msg_.poses[i].position.x = p_pose.x();
            particle_cloud_msg_.poses[i].position.y = p_pose.y();
            particle_cloud_msg_.poses[i].position.z = 0.0;

            // 角度 (yaw) を Quaternion に変換して代入
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, p_pose.yaw());
            particle_cloud_msg_.poses[i].orientation = tf2::toMsg(q);
        }

        // 4. パブリッシュ
        pub_particle_cloud_->publish(particle_cloud_msg_);
    }
}

#include <tf2/utils.h>

double Localizer::get_yaw_from_quat(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
}

// 1. パーティクルを初期化する関数（数値3つを受け取る）
void Localizer::initialize_particles(double x, double y, double yaw)
{
    if (particles_.empty()) {
        particles_.resize(max_particle_num_);
    }

    for (auto& p : particles_) {
        // パラメータの標準偏差を使って散らす
        double px = norm_rv(x, init_x_dev_);
        double py = norm_rv(y, init_y_dev_);
        double pyaw = norm_rv(yaw, init_yaw_dev_);
        p.pose_.set(px, py, pyaw); 
    }
    reset_weight(); 
}

// 2. RVizからの初期位置を受け取るコールバック（メッセージを受け取る）
void Localizer::callback_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Initial pose received from RViz!");

    // メッセージから数値を取り出す
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = get_yaw_from_quat(msg->pose.pose.orientation);

    // 数値を渡してパーティクルを初期化
    initialize_particles(x, y, yaw);

    // 推定位置も更新
    estimated_pose_.set(x, y, yaw);
    flag_broadcast_ = true; 
}