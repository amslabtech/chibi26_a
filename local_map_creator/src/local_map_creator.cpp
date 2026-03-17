#include "local_map_creator/local_map_creator.hpp"
#include "math.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creater")
{
    // パラメータの取得(hz, map_size, map_reso)
    this->declare_parameter("hz", 10);
    this->declare_parameter("map_size", 100.0);
    this->declare_parameter("map_reso", 0.1);

    hz_ = this->get_parameter("hz").as_int();
    map_size_ = this->get_parameter("map_size").as_double();
    map_reso_ = this->get_parameter("map_reso").as_double();

    // Subscriberの設定
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("obs", 10, std::bind(&LocalMapCreator::obs_poses_callback, this, std::placeholders::_1));
    // Publisherの設定
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
    
    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header
    local_map_.header.frame_id = "base_link"; // ロボット中心のマップなら base_link

    //   info(width, height, position.x, position.y)
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = map_size_;
    local_map_.info.height = map_size_;

    // ロボットをマップの中心に配置
    local_map_.info.origin.position.x = -(map_size_ * map_reso_) / 2.0;
    local_map_.info.origin.position.y = -(map_size_ * map_reso_) / 2.0;
    local_map_.info.origin.position.z = 0.0;
    local_map_.info.origin.orientation.w = 1.0;

    //   data
    init_map();
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq()
{
    return hz_;
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process()
{
    if(! flag_obs_poses_) return;

    update_map();
}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map()
{
    // マップを初期化する
    std::fill(local_map_.data.begin(), local_map_.data.end(), 0);

    // 障害物の位置を考慮してマップを更新する
    for(const auto &pose : obs_poses_.poses){
        int index = xy_to_grid_index(pose.position.x, pose.position.y);

        // マップの範囲内（有効なインデックス）であれば「100：占有」を書き込む
        if(index != -1){
            local_map_.data[index] = 100;
        }
    }

    // 更新したマップをpublishする
    local_map_.header.stamp = this->now();// タイムスタンプ
    pub_local_map_->publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
    local_map_.data.assign(map_size_ * map_size_, -1); //初期値（すべて「未知：-1」で埋める）
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    // 指定された距離と角度がマップの範囲内か判定する
    double x = dist * std::cos(angle);
    double y = dist * std::sin(angle);

    int index = xy_to_grid_index(x, y);

    return (index != -1);// インデックスが -1 でなければマップ内
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
    double x = dist * std::cos(angle);
    double y = dist * std::sin(angle);

    return xy_to_grid_index(x, y);
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y)
{
    // マップの左下(origin)からの相対座標に変換
    double rel_x = x - local_map_.info.origin.position.x;
    double rel_y = y - local_map_.info.origin.position.y;

    //　解像度で割って「何マス目(x, y)」にいるかを算出
    int gx = std::floor(rel_x / map_reso_);
    int gy = std::floor(rel_y / map_reso_);

    // マップの範囲外チェック
    if(gx < 0 || gx >= (int)map_size_ || gy < 0 || gy >= (int)map_size_){
        return -1; //範囲外なら-1を返す
    }
    
    return gy * (int)map_size_ + gx;
}
