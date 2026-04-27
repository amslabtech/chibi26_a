#include "obstacle_detector.hpp"
#include <chrono>
#include <limits>

//㎳単位を使用するため
using namespace std::chrono_literals;

ObstacleDetector::ObstacleDetector()
: Node("a_obstacle_detector")
{
   // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
    this->declare_parameter("hz", 10);
    this->declare_parameter("ignore_dist", 0.5);
    this->declare_parameter("laser_step", 1);
    this->declare_parameter("robot_frame", "base_link");

    hz_ = this->get_parameter("hz").as_int();
    obs_dist = this->get_parameter("ignore_dist").as_double();
    int laser_step = this->get_parameter("laser_step").as_int();
    robot_frame = this->get_parameter("robot_frame").as_string();

    //sub && pub && timer
    // timer_ = this->create_timer(this->get_clock(), std::chrono::milliseconds(1000 / hz_), std::bind(&ObstacleDetector::process, this));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / hz_) ,std::bind(&ObstacleDetector::process, this));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));
    obs_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacle_points", 10);
}

//Lidarから障害物の情報を取得
void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_scan_ = true;
}

//一定周期で行う処理(obstacle_detectorの処理)
void ObstacleDetector::process()
{
    //データがないときはスキップ
    if(! flag_scan_) return;
    //scan_obstacle()を呼び出す
    scan_obstacle();  
}

//Lidarから障害物情報を取得し，障害物の座標をpublish
void ObstacleDetector::scan_obstacle()
{
    // 送信用の PoseArray メッセージを作成
    auto message = geometry_msgs::msg::PoseArray();
    message.header = laser_->header; // 元のScanのヘッダー（時刻やframe_id）をコピー
    // if (message.header.frame_id.empty()) {
    //     message.header.frame_id = robot_frame;
    // }

    int step = this->get_parameter("laser_step").as_int();

    for(int i = 0; i < (int)laser_->ranges.size(); i += step)
    {
        // 無視すべき範囲（柱など）ではない場合のみ処理
        if(!is_ignore_scan(i))
        {
            float range = laser_->ranges[i];
            float angle = laser_->angle_min + (i * laser_->angle_increment);

            // 極座標 (range, angle) から直交座標 (x, y) へ変換
            geometry_msgs::msg::Pose pose;
            pose.position.x = range * std::cos(angle);
            pose.position.y = range * std::sin(angle);
            pose.position.z = 0.0;

            // 配列に追加
            message.poses.push_back(pose);
        }
    }

    // 型が一致した状態でパブリッシュ！
    obs_pub_->publish(message);
}


//無視するlidar情報の範囲の決定(lidarがroombaの櫓の中にあり，櫓の４つの柱を障害物として検出してしまうため削除が必要)
bool ObstacleDetector::is_ignore_scan(int index)
{
    float range = laser_->ranges[index];
    // 距離が0（エラー値）や範囲外の場合は無視
    if(range < laser_->range_min || laser_->range_max < range) return true;

    // 現在のインデックスの角度を計算 (rad)
    float angle = laser_->angle_min + (index * laser_->angle_increment);

    // 柱を無視する（角度 ＋ 距離の条件を追加）
    // float ignore_dist = 0.81; // 柱があると思われる最大距離<-yamlファイルからobs_distとして入力している

    if(range < obs_dist) { 
        //右前の柱を無視＜45度=0.785rad（0.7rad ~ 0.85rad）付近＞
        if((angle > 0.7 && angle < 0.85) && range < obs_dist) return true;

        //右後の柱を無視＜135度=2.356rad（2.2rad ~ 2.4rad）付近＞
        if((angle > 2.2 && angle < 2.4) && range < obs_dist) return true;

        //左後の柱を無視＜-45度=-0.785rad（-0.85rad ~ -0.7rad）付近＞
        if((angle > -0.85 && angle < -0.7) && range < obs_dist) return true;

        //左前の柱を無視＜-135度=-2.356rad（-2.4rad ~ -2.0rad）付近＞
        if((angle > -2.4 && angle < -2.0) && range < obs_dist) return true;
    }
    // if((angle > 0.7 && angle < 0.85) && range < obs_dist) return true;
    //     //右後の柱を無視＜135度=2.356rad（2.2rad ~ 2.4rad）付近＞
    // if((angle > 2.2 && angle < 2.4) && range < obs_dist) return true;
    //     //左後の柱を無視＜-45度=-0.785rad（-0.85rad ~ -0.7rad）付近＞
    // if((angle > -0.85 && angle < -0.7) && range < obs_dist) return true;
    //     //左前の柱を無視＜-135度=-2.356rad（-2.4rad ~ -2.0rad）付近＞
    // if((angle > -2.4 && angle < -2.0) && range < obs_dist) return true;

    return false;
}