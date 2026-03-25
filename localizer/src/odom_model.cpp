#include "localizer/odom_model.hpp"

// デフォルトコンストラクタ
OdomModel::OdomModel() : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0){}

// コンストラクタ
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
{
    fw_var_per_fw_ = ff;
    fw_var_per_rot_ = rf;
    rot_var_per_fw_ = fr;
    rot_var_per_rot_ = rr;
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{
    if (this != &model) {
        this->fw_var_per_fw_  = model.fw_var_per_fw_;
        this->fw_var_per_rot_ = model.fw_var_per_rot_;
        this->rot_var_per_fw_  = model.rot_var_per_fw_;
        this->rot_var_per_rot_ = model.rot_var_per_rot_;
        this->fw_dev_  = model.fw_dev_;
        this->rot_dev_ = model.rot_dev_;
        // 乱数エンジンはコピーしない（または状態をコピーする）のが一般的
    }
    return *this;
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    // 移動の絶対値を取得
    double l = std::abs(length);
    double a = std::abs(angle);

    // 分散 = (直進による誤差) + (回転による誤差)
    // 分散から標準偏差 (sqrt) を求める
    fw_dev_  = std::sqrt(l * fw_var_per_fw_  + a * fw_var_per_rot_);
    rot_dev_ = std::sqrt(l * rot_var_per_fw_ + a * rot_var_per_rot_);
}

// 直進に関するノイズ（fw_dev_）の取得
double OdomModel::get_fw_noise()
{
    // 標準正規分布(0, 1)から得た値に標準偏差をかける
    return std_norm_dist_(engine_) * fw_dev_;
}

// 回転に関するノイズ（rot_dev_）の取得
double OdomModel::get_rot_noise()
{
    return std_norm_dist_(engine_) * rot_dev_;
}
