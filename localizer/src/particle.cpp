#include "localizer/particle.hpp"
#include <cmath>

// デフォルトコンストラクタ
Particle::Particle() : pose_(0.0, 0.0, 0.0), weight_(0.0)
{

}

// コンストラクタ
Particle::Particle(const double x, const double y, const double yaw, const double weight) : pose_(x, y, yaw), weight_(weight)
{

}

// 代入演算子
Particle& Particle::operator =(const Particle& p)
{
        pose_ = p.pose_;
        weight_ = p.weight_;
        return *this;
}

// setter
void Particle::set_weight(const double weight)
{
        weight_ = weight;
}

// 尤度関数
// センサ情報からパーティクルの姿勢を尤度を算出
double Particle::likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser,
        const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list)
{
    double L = 0.0; // 合計尤度
    int count = 0;

    // レーザーの各ビームについてループ（laser_stepごとに間引く）
    for (size_t i = 0; i < laser.ranges.size(); i += laser_step)
    {
        double angle = laser.angle_min + i * laser.angle_increment;
        
        // 特定の角度（ロボットの柱など）を無視する処理
        if (is_ignore_angle(angle, ignore_angle_range_list)) continue;

        // センサーの実測値（無限遠やエラー値はスキップ）
        double d_sensor = laser.ranges[i];
        if (std::isnan(d_sensor) || std::isinf(d_sensor)) continue;

        // パーティクルの姿勢から見た、このビームの絶対角度
        double global_angle = pose_.yaw() + angle;

        // 地図上での壁までの距離をシミュレーション
        double d_map = calc_dist_to_wall(pose_.x(), pose_.y(), global_angle, map, laser.range_max, sensor_noise_ratio);

        // 誤差を正規分布で評価
        // 誤差が小さいほど大きな値（確率密度）が返る
        // 標準偏差（stddev）として sensor_noise_ratio などを使用
        double p = norm_pdf(d_sensor, d_map, sensor_noise_ratio);

        // 尤度の更新（積だとすぐ0になるため、対数をとって足すのが一般的）
        // 単純な実装なら L += p; でも動きます（その場合は平均をとる）
        L += p; 
        count++;
    }

    // 平均尤度を返す
    return (count > 0) ? (L / count) : 0.0;
}

// 柱がある範囲か判定
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{
        // 角度を -PI ~ PI の範囲に正規化（念のため）
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        // リストが [start1, end1, start2, end2, ...] のペアで構成されている前提
        for (size_t i = 0; i + 1 < ignore_angle_range_list.size(); i += 2)
        {
                double start = ignore_angle_range_list[i];
                double end   = ignore_angle_range_list[i+1];

                // 角度が範囲内（start <= angle <= end）にあるかチェック
                if (angle >= start && angle <= end)
                {
                        return true; // 除外対象
                }
        }

        return false; // 有効な角度
}

// 与えられた座標と角度の方向にある壁までの距離を算出
// マップデータが100の場合，距離を返す
// マップデータが-1（未知）の場合，マップ範囲外の場合はsearch_limit * 2.0を返す
// いずれでもない場合は，search_limitを返す
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio)
{
        (void)sensor_noise_ratio;

        // 探索のステップサイズ
        const double search_step = map.info.resolution;
        // 最大探索距離
        const double search_limit = laser_range;

        // 探索
        for(double dist=0.0; dist<search_limit; dist+=search_step)
        {
                double nx = x + dist * std::cos(laser_angle);
                double ny = y + dist * std::sin(laser_angle);

                double relative_x = nx - map.info.origin.position.x;
                double relative_y = ny - map.info.origin.position.y;

                int grid_x = static_cast<int>(std::floor(relative_x / map.info.resolution));
                int grid_y = static_cast<int>(std::floor(relative_y / map.info.resolution));

                if (!in_map_cell(grid_x, grid_y, map.info)) {
                        return search_limit * 2.0;
                }

                int index = grid_y * static_cast<int>(map.info.width) + grid_x;

                int8_t cell_value = map.data[index];

                if (cell_value == 100) {
                        return dist;
                }
                else if (cell_value == -1) {
                        return search_limit * 2.0;      
                }

        }
        
        return search_limit;
}

// 座標からグリッドのインデックスを返す
int Particle::xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info)
{
        // 1. 地図の左下原点からの相対距離を計算
        double relative_x = x - map_info.origin.position.x;
        double relative_y = y - map_info.origin.position.y;

        // 2. メートル単位をグリッド単位（マス目）に変換
        int grid_x = static_cast<int>(std::floor(relative_x / map_info.resolution));
        int grid_y = static_cast<int>(std::floor(relative_y / map_info.resolution));

        // 3. 範囲外チェック（オプション：in_map関数でやるならここは計算のみ）
        if (grid_x < 0 || grid_x >= static_cast<int>(map_info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(map_info.height))
        {
        return -1; // 範囲外を示すインデックス
        }

        // 4. 1次元配列のインデックスを返す
        return grid_y * map_info.width + grid_x;
}

bool Particle::in_map_cell(int grid_x, int grid_y, const nav_msgs::msg::MapMetaData& map_info)
{
    return grid_x >= 0 &&
           grid_x < static_cast<int>(map_info.width) &&
           grid_y >= 0 &&
           grid_y < static_cast<int>(map_info.height);
}

// マップ内か判定
bool Particle::in_map(const int grid_index, const int map_data_size)
{
        // インデックスが0以上、かつ配列の要素数より小さければ地図内
        return (grid_index >= 0 && grid_index < map_data_size);
}

// 確率密度関数（正規分布）
double Particle::norm_pdf(const double x, const double mean, const double stddev)
{
        double std = std::max(stddev, 1e-6);
        double diff = x - mean;
        return std::exp(-(diff * diff) / (2.0 * std * std));
}
