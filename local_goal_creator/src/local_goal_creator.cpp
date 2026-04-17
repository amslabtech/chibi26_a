#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    //pubやsubの定義，tfの統合
    //初期値の設定
    // ループ周期 [Hz]
    // １回で更新するインデックス数
    // グローバルパス内におけるローカルゴールのインデックス
    // 現在位置-ゴール間の距離 [m]

    hz_ = 10; //周期
    index_step_ = 1; //1回で更新するインデックス数
    goal_index_ = 0; //グローバルパス内におけるローカルゴールのインデックス
    target_distance_ = 2.0; // ローカルゴールとする前方距離 [m]
    is_path_ = false;

    // Subscriberの定義
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path", 10, std::bind(&LocalGoalCreator::pathCallback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose", 10, std::bind(&LocalGoalCreator::poseCallback, this, std::placeholders::_1));
        // current
    // Publisherの定義
    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("local_goal", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz
        std::bind(&LocalGoalCreator::process, this)
    );

    RCLCPP_INFO(this->get_logger(), "Local Goal Creator Node has been started.");

}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_ = *msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_ = *msg;     // 届いた「経路データ」を変数 path_ に丸ごと保存する
    is_path_ = true;  // 「パスを受け取ったよ」というフラグを立てる
    goal_index_ = 0;  // 新しい経路が来たら、また最初(0)から追いかけ直す
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
    if(is_path_ && !path_.poses.empty())
    {
        publishGoal();
    }
}

void LocalGoalCreator::publishGoal()
{
    //ゴールまでの距離の計算を行う
    //設定値に応じて，ゴール位置の変更を行う
    // 現在のgoal_indexから先のパスを探索

    // pose_.pose.position.x = 0.0;  <-コメントアウト
    // pose_.pose.position.y = 0.0;

    // 1. 現在のゴール(goal_index_)とロボットの距離を計算
    double dist_to_goal = getDistance();

    // 2. もし設定した距離（target_distance_）より近くなったら、ゴールをパスに沿って進める
    // パスの終端(size() - 1)に達するまでループ
    while (dist_to_goal < target_distance_ && goal_index_ < (int)path_.poses.size() - 1) 
    {
        goal_index_ += index_step_; //ターゲットを1つ先に進める
        
        dist_to_goal = getDistance(); //進めた後の新しい距離を測り直す
    }

    // 3. ローカルゴールのメッセージを作成して配信
    goal_.header.stamp = this->now();
    goal_.header.frame_id = path_.header.frame_id; // パスと同じ座標系を使用
    goal_.point = path_.poses[goal_index_].pose.position;

    local_goal_pub_->publish(goal_);
}

double LocalGoalCreator::getDistance()//距離計算関数（使わなくても平気）
{
    // 現在地と現在のローカルゴールとの距離を返す
    // ロボットの現在位置と、パス上の goal_index_ 番目の点との距離を計算
    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    
    return std::hypot(dx, dy);
}