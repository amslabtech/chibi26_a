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
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Localizer::scan_callback, this, std::placeholders::_1));

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

    odom_model_.set_parameters(ff, fr, rf, rr);

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
    map_ = *msg_

}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 初回実行時の処理
    if (!last_odom_received_) {
        last_odom_ = *msg;
        last_odom_received_ = true;
        return;
    }

    //前回からの移動量（差分）を計算
    // 現在の姿勢 (Quaternion -> Yaw)
    double curr_yaw = get_yaw_from_quat(msg->pose.pose.orientation);
    double last_yaw = get_yaw_from_quat(last_odom_.pose.pose.orientation);

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
        // odom_modelを使って各パーティクルにノイズを乗せて移動
        for (auto& p : particle_cloud_msg_.poses) {
            odom_model_.update_particle(p, dx, dy, dth);
        }
        // 更新したら今回の値を保存
        last_odom_ = *msg;
    }
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ = *msg;
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

    
    Particle particle;

    // 初期位置近傍にパーティクルを配置

    // パーティクルの重みの初期化
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_)
    {

    }

}

// 適切な角度(-M_PI ~ M_PI)を返す
double Localizer::normalize_angle(double angle)
{

}

// ランダム変数生成関数（正規分布）
double Localizer::norm_rv(const double mean, const double stddev)
{
    
}

// パーティクルの重みの初期化
void Localizer::reset_weight()
{

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

        // odom座標系からみたbase_link座標系の位置と姿勢の取得

        // map座標系からみたodom座標系の位置と姿勢を計算（回転行列を使った単純な座標変換）

        // yawからquaternionを作成
        tf2::Quaternion map_to_odom_quat;

        // odom座標系odomの位置姿勢情報格納するための変数
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納

        // 親フレーム・子フレームの指定
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

        // map座標系からみたodom座標系の原点位置と方向の格納

        // tf情報をbroadcast(座標系の設定)
    }

}

// 自己位置推定
// 動作更新と観測更新を行う
void Localizer::localize()
{

}

// 動作更新
// ロボットの微小移動量を計算し，パーティクルの位置をノイズを加えて更新
void Localizer::motion_update()
{

}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    // パーティクル1つのレーザ1本における平均尤度を算出
    const double alpha;
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{

}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
void Localizer::estimate_pose()
{
    
}

// 重みの正規化
void Localizer::normalize_belief()
{

}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting()
{

}

// リサンプリング（系統サンプリング）
// 周辺尤度に応じてパーティクルをリサンプリング
void Localizer::resampling(const double alpha)
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    int size = particles_.size();

    // particle数の動的変更

    // サンプリングするパーティクルのインデックスを保持

    // リサンプリング

    // 重みを初期化
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{

}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    if(is_visible_)
    {
        
    }
}