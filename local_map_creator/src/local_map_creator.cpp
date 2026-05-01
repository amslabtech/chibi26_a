#include "local_map_creator/local_map_creator.hpp"
#include "math.h"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creater")
{
    // パラメータの取得(hz, map_size, map_reso)
    this->declare_parameter("hz", 10);
    this->declare_parameter("map_size", 4.0);
    this->declare_parameter("map_reso", 0.01);

    hz_ = this->get_parameter("hz").as_int();
    map_size_ = this->get_parameter("map_size").as_double();
    map_reso_ = this->get_parameter("map_reso").as_double();

    // Subscriberの設定
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("obstacle_points", 10, std::bind(&LocalMapCreator::obs_poses_callback, this, std::placeholders::_1));
    // Publisherの設定
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
    
    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header
    // マス数を計算（重要！）
    uint32_t grid_width = static_cast<uint32_t>(map_size_ / map_reso_);
    local_map_.header.frame_id = "base_link"; // ロボット中心のマップなら base_link

    //   info(width, height, position.x, position.y)
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = grid_width;
    local_map_.info.height = grid_width;

    // ロボットをマップの中心に配置
    local_map_.info.origin.position.x = -(map_size_ /** map_reso_*/) / 2.0;
    local_map_.info.origin.position.y = -(map_size_ /** map_reso_*/) / 2.0;
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
    std::fill(local_map_.data.begin(), local_map_.data.end(), -1);

    int start_gx = (int)local_map_.info.width / 2;
    int start_gy = (int)local_map_.info.height / 2;

    // 障害物の位置を考慮してマップを更新する
    for(const auto &pose : obs_poses_.poses){
        // int index = xy_to_grid_index(pose.position.x, pose.position.y);
        
        // // マップの範囲内（有効なインデックス）であれば「100：占有」を書き込む
        // if(index != -1){
        //     local_map_.data[index] = 100;
        // }

        double dist = std::hypot(pose.position.x, pose.position.y);
        
        // 障害物の座標をグリッド単位に変換
        int end_gx = std::floor((pose.position.x - local_map_.info.origin.position.x) / map_reso_);
        int end_gy = std::floor((pose.position.y - local_map_.info.origin.position.y) / map_reso_);

        // 2. 【追加】中心からその点までを「白（0：空き）」で塗りつぶす
        raytrace(start_gx, start_gy, end_gx, end_gy);

        // 3. 【修正】本当に障害物（柱より遠い）場合だけ「黒（100）」を置く
        // 柱の除去距離（0.81m）より遠いものだけを描画
        if (dist >= 0.81) { 
            int index = xy_to_grid_index(pose.position.x, pose.position.y);
            if (index != -1) {
                local_map_.data[index] = 100;
            }
        }
    }

    // 更新したマップをpublishする
    local_map_.header.stamp = this->now();// タイムスタンプ
    pub_local_map_->publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
    local_map_.data.assign(local_map_.info.width * local_map_.info.height/*map_size_ * map_size_*/, -1); //初期値（すべて「未知：-1」で埋める）
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
    if(gx < 0 || gx >= (int)local_map_.info.width/*map_size_*/ || gy < 0 || gy >= (int)local_map_.info.height/*map_size_*/){
        return -1; //範囲外なら-1を返す
    }
    
    return gy * (int)local_map_.info.width/*map_size_*/ + gx;
}

void LocalMapCreator::raytrace(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int width = (int)local_map_.info.width;
    int height = (int)local_map_.info.height;

    while (true) {
        // 終点（障害物があるマス）に到達したら終了
        if (x0 == x1 && y0 == y1) break;

        // マップの範囲内なら「0：空き」を書き込む
        if (x0 >= 0 && x0 < width/*(int)map_size_*/ && y0 >= 0 && y0 < height/*(int)map_size_*/) {
            local_map_.data[y0 * width/*(int)map_size_*/ + x0] = 0;
        } else {
            break; // マップ外に出たら終了
        }

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}
