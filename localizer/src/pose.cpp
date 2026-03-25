#include "localizer/pose.hpp"
#include <cmath>

// デフォルトコンストラクタ
Pose::Pose(): x_(0.0), y_(0.0), yaw_(0.0) {}

// コンストラクタ
Pose::Pose(const double x, const double y, const double yaw): x_(x), y_(y), yaw_(yaw)
{
    normalize_angle();
}

// 代入演算子
Pose& Pose::operator =(const Pose& pose)
{
    if (this != &pose) {
        this->x_ = pose.x_;
        this->y_ = pose.y_;
        this->yaw_ = pose.yaw_;
    }
    return *this;
}

// 複合代入演算子/=
Pose& Pose::operator /=(const double a)
{
    if (std::abs(a) > 1e-9) {
        this->x_ /= a;
        this->y_ /= a;
        this->yaw_ /= a;
    }
    return *this;
}

// setter
void Pose::set(const double x, const double y, const double yaw)
{
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = yaw;
    normalize_angle();
}

// パーティクルの移動
// ノイズを加えて，移動させる
void Pose::move(double length, double direction, double rotation, const double fw_noise, const double rot_noise)
{
    // 移動方向（現在の向き + 移動の向き）
    double move_angle = this->yaw_ + direction;

    // 座標の更新（ノイズを加味した移動）
    // fw_noise や rot_noise は外部（OdomModel）で計算されたランダム値が渡される想定
    this->x_ += (length + fw_noise) * std::cos(move_angle);
    this->y_ += (length + fw_noise) * std::sin(move_angle);

    // 向きの更新
    this->yaw_ += rotation + rot_noise;

    // 更新後の角度を正規化
    normalize_angle();
}

// 適切な角度(-M_PI ~ M_PI)に変更
void Pose::normalize_angle()
{
    while (this->yaw_ >  M_PI) this->yaw_ -= 2.0 * M_PI;
    while (this->yaw_ < -M_PI) this->yaw_ += 2.0 * M_PI;
}