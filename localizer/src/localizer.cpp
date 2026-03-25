#include "localizer/localizer.hpp"

// デフォルトコンストラクタ
// パラメータの宣言と取得
// Subscriber，Publisherの設定
// frame idの設定
// パーティクルクラウドのメモリの確保
// odometryのモデルの初期化
Localizer::Localizer() : Node("team_localizer")
{ 
    // パラメータの宣言
    this->declare_parameter("hz", 10);
    this->declare_parameter("max_particle_num", 500);
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_yaw", 0.0);

    // パラメータの取得
    this->get_parameter("hz", hz_);
    this->get_parameter("max_particle_num", max_particle_num_);

    // Subscriberの設定
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&Localizer::map_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Localizer::odom_callback, this, std::placeholders::_1));
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Localizer::laser_callback, this, std::placeholders::_1));

    // Publisherの設定
    pub_estimated_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);
    pub_particle_cloud_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);

    // frame idの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";

    // パーティクルクラウドのメモリの確保
    particle_cloud_msg_.poses.reserve(max_particle_num_);

    // odometryのモデルの初期化
    this->declare_parameter("ff", 0.17);
    this->declare_parameter("fr", 0.0005);
    this->declare_parameter("rf", 0.13);
    this->declare_parameter("rr", 0.2);

    double ff, fr, rf, rr;
    this->get_parameter("ff", ff);
    this->get_parameter("fr", fr);
    this->get_parameter("rf", rf);
    this->get_parameter("rr", rr);

    odom_model_ = OdomModel(ff, fr, rf, rr);

    double i_x, i_y, i_yaw;
    this->get_parameter("init_x", i_x);
    this->get_parameter("init_y", i_y);
    this->get_parameter("init_yaw", i_yaw);

    initialize_particles(i_x, i_y, i_yaw);

    RCLCPP_INFO(this->get_logger(), "Localizer: Initialized with YAML parameters.");

}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;

}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 初回実行時の処理
    if (!flag_odom_) {
        last_odom_ = *msg;
        flag_odom_ = true;
        return;
    }

    //前回からの移動量（差分）を計算
    // 現在の姿勢 (Quaternion -> Yaw)
    double curr_yaw = Localizer::get_yaw_from_quat(msg->pose.pose.orientation);
    double last_yaw = Localizer::get_yaw_from_quat(last_odom_.pose.pose.orientation);

    // 位置の差分（グローバル座標系での差）
    double dx = msg->pose.pose.position.x - last_odom_.pose.pose.position.x;
    double dy = msg->pose.pose.position.y - last_odom_.pose.pose.position.y;
    double dth = curr_yaw - last_yaw;

    // 角度の正規化 (-PI ~ PI)
    while (dth >  M_PI) dth -= 2.0 * M_PI;
    while (dth < -M_PI) dth += 2.0 * M_PI;

    //各パーティクルを移動させる
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist > move_dist_th_ || std::abs(dth) > 0.05) {

        // 4. OdomModel を使ってノイズを取得し、各パーティクルを移動
        // (進行方向の角度を計算: ロボット前進方向からの相対角)
        double relative_direction = std::atan2(dy, dx) - last_yaw;

        // モデルに移動量をセットしてノイズを準備
        odom_model_.set_dev(dist, dth);

        for (auto& p : particles_) { // メッセージではなく vector<Particle> を回す
            double fw_noise = odom_model_.get_fw_noise();
            double rot_noise = odom_model_.get_rot_noise();
            
            // Particleクラスの move 関数を呼び出す (p.pose_ は public)
            p.pose_.move(dist, relative_direction, dth, fw_noise, rot_noise);
        }

        // 更新したら今回の値を保存
        last_odom_ = *msg;
    }
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
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
    
    Particle particle;

    // 初期位置近傍にパーティクルを配置
    for (int i = 0; i < max_particle_num_; i++) {
        // 初期位置の周囲に正規分布で散らす
        double px = norm_rv(i_x, 0.2);
        double py = norm_rv(i_y, 0.2);
        double pyaw = norm_rv(i_yaw, 0.1);
        particles_.emplace_back(px, py, pyaw, 1.0 / max_particle_num_);
    }
    // パーティクルの重みの初期化
    estimated_pose_.set(i_x, i_y, i_yaw);
    reset_weight();
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_)
    {
        localize();             // 自己位置推定実行
        broadcast_odom_state(); // TF配信
        publish_particles();    // パーティクル表示
        publish_estimated_pose(); // 推定位置パブリッシュ
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
    static std::mt19937 engine(std::random_device{}());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(engine);
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
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

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
    // 1. 前回のオドメトリ受信時からの差分を計算
    // 現在の姿勢 (Quaternion -> Yaw)
    double curr_yaw = get_yaw_from_quat(prev_odom_.pose.pose.orientation);
    double last_yaw = get_yaw_from_quat(last_odom_.pose.pose.orientation);

    // 位置の差分（グローバル座標系での単純差分）
    double dx = prev_odom_.pose.pose.position.x - last_odom_.pose.pose.position.x;
    double dy = prev_odom_.pose.pose.position.y - last_odom_.pose.pose.position.y;
    double dth = normalize_angle(curr_yaw - last_yaw);

    // 2. 移動距離の計算
    double dist = std::sqrt(dx * dx + dy * dy);

    // 3. 一定以上の移動があった場合のみパーティクルを更新（計算負荷軽減）
    if (dist > move_dist_th_ || std::abs(dth) > move_angle_th_) 
    {
        // オドメトリモデルに現在の移動量をセットして、ノイズの標準偏差を計算
        odom_model_.set_dev(dist, dth);

        // 各パーティクルを移動させる
        for (auto& p : particles_) 
        {
            // odom_modelから生成したノイズを取得
            double fw_noise = odom_model_.get_fw_noise();
            double rot_noise = odom_model_.get_rot_noise();

            // パーティクルの持つ Pose クラスの move 関数を呼び出す
            // dx, dy から移動の方向 (atan2) を算出
            double move_direction = std::atan2(dy, dx);
            
            // 注意: move_direction は global 座標系での移動方向。
            // パーティクルの現在の向き (p.yaw) との相対角にする必要がある場合は調整
            double relative_direction = normalize_angle(move_direction - last_yaw);

            p.pose_.move(dist, relative_direction, dth, fw_noise, rot_noise);
        }

        // 今回のオドメトリを「前回の値」として保存
        last_odom_ = prev_odom_;
    }
}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    double total_weight = 0.0;
    // パーティクル1つのレーザ1本における平均尤度を算出
    for (auto& p : particles_) {
        double w = p.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
        p.set_weight(w);
        total_weight += w;
    }

    // 重みの正規化
    if (total_weight > 0.0) {
        for (auto& p : particles_) {
            p.set_weight(p.weight() / total_weight);
        }
    } else {
        reset_weight();
    }

    // 推定位置の決定
    estimate_pose();

    // リサンプリング（周辺尤度 alpha を計算して渡す）
    double alpha = total_weight / particles_.size();
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
void Localizer::expansion_resetting()
{
    // 周辺尤度（平均尤度）を計算
    double alpha = calc_marginal_likelihood();

    // しきい値（例: 0.01）を下回った場合にリセット発動
    // ※しきい値は環境やセンサー精度に合わせて調整が必要
    if (alpha < expansion_threshold_) 
    {
        RCLCPP_WARN(this->get_logger(), "Expansion Resetting Triggered! (alpha: %f)", alpha);

        for (auto& p : particles_) {
            // 現在の推定位置を中心に、通常より大きなノイズを加えて再配置
            double px = norm_rv(estimated_pose_.x(), 0.5); // 標準偏差 50cm
            double py = norm_rv(estimated_pose_.y(), 0.5);
            double pyaw = norm_rv(estimated_pose_.yaw(), 0.3); // 約17度

            p.pose_.set(px, py, pyaw);
        }
        
        // リセット後は重みを均一化
        reset_weight();
    }
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
        // 1. ヘッダー情報の更新（最新の時刻とフレームID）
        particle_cloud_msg_.header.stamp = this->now();
        particle_cloud_msg_.header.frame_id = map_.header.frame_id;

        // 2. 現在のパーティクル数に合わせて PoseArray のサイズを調整
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

// パーティクルを初期化する関数
void Localizer::initialize_particles(double x, double y, double yaw)
{
    // 既存の particles_ が空だとループが回らないので、
    // コンストラクタで指定した max_particle_num_ 分だけリサイズして確保
    if (particles_.empty()) {
        particles_.resize(max_particle_num_);
    }

    for (auto& p : particles_) {
        p.pose_ = Pose(x, y, yaw); 
    }
    
    // すでに定義されている関数を使って重みを一括初期化（private問題を回避）
    reset_weight(); 
}