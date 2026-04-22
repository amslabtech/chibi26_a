#include "global_path_planner/global_path_planner.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// デフォルトコンストラクタ
Astar::Astar() : Node("a_global_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言と取得 ######
    this->declare_parameter("margin", 0.3);
    this->declare_parameter("way_points_x", std::vector<double>{0.0, 16.4, 16.2, -17.3, -17.2, 0.0});  //修正
    this->declare_parameter("way_points_y", std::vector<double>{0.0, 0.221, 14.2, 13.8, 0.0946, 0.0});
    this->declare_parameter("test_show", false);
    this->declare_parameter("sleep_time", 0.01);

    margin_ = this->get_parameter("margin").as_double();
    way_points_x_ = this->get_parameter("way_points_x").as_double_array();
    way_points_y_ = this->get_parameter("way_points_y").as_double_array();
    test_show_ = this->get_parameter("test_show").as_bool();
    sleep_time_ = this->get_parameter("sleep_time").as_double();

    // ###### frame_id設定 ######
    global_path_.header.frame_id = "map";
    current_node_.header.frame_id = "map";

    // dataサイズの確保
    global_path_.poses.reserve(2000);

    // ####### Subscriber #######
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&Astar::map_callback, this, std::placeholders::_1));

    // ###### Publisher ######
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    pub_node_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("node_point", 10);
    pub_current_path_ = this->create_publisher<nav_msgs::msg::Path>("current_path", 10);
    pub_new_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_map", 10);
}

// mapのコールバック関数
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (map_checker_) return;
    map_ = *msg;
    new_map_ = *msg;
    width_ = msg->info.width;
    height_ = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    map_checker_ = true;
    process();
}

void Astar::process()
{
    if(!map_checker_){
        RCLCPP_INFO(this->get_logger(), "NOW LOADING...");
    } else {
        RCLCPP_INFO(this->get_logger(), "NOW LOADED MAP");
        obs_expander();
        planning();
    }
}

// マップ全体の障害物を拡張処理
void Astar::obs_expander()
{
    for (int i = 0; i < width_ * height_; ++i) {
        if (map_.data[i] > 50) obs_expand(i);
    }
    pub_new_map_->publish(new_map_);
}

void Astar::obs_expand(const int index)
{
    int m_grid = static_cast<int>(margin_ / resolution_);
    int cx = index % width_;
    int cy = index / width_;
    for (int dy = -m_grid; dy <= m_grid; ++dy) {
        for (int dx = -m_grid; dx <= m_grid; ++dx) {
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                new_map_.data[ny * width_ + nx] = 100;
            }
        }
    }
}

double Astar::make_heuristic(const Node_ node)
{
    return std::hypot(goal_node_.x - node.x, goal_node_.y - node.y);
}

Node_ Astar::set_way_point(int phase)
{
    Node_ node;
    node.x = static_cast<int>((way_points_x_[phase] - origin_x_) / resolution_);
    node.y = static_cast<int>((way_points_y_[phase] - origin_y_) / resolution_);
    return node;
}

void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    Node_ current = node;
    while (current.parent_x != -1) {
        partial_path.poses.push_back(node_to_pose(current));
        int idx = search_node_from_list({current.parent_x, current.parent_y}, close_list_);
        if (idx != -1) current = close_list_[idx];
        else break;
    }
    std::reverse(partial_path.poses.begin(), partial_path.poses.end());
    global_path_.poses.insert(global_path_.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
    pub_path_->publish(global_path_);
}

geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = node.x * resolution_ + origin_x_;
    pose.pose.position.y = node.y * resolution_ + origin_y_;
    pose.pose.orientation.w = 1.0;
    return pose;
}

Node_ Astar::select_min_f()
{
    auto it = std::min_element(open_list_.begin(), open_list_.end(), 
              [](const Node_& a, const Node_& b){ return a.f < b.f; });
    return *it;
}

bool Astar::check_start(const Node_ node) { return check_same_node(node, start_node_); }
bool Astar::check_goal(const Node_ node) { return check_same_node(node, goal_node_); }
bool Astar::check_same_node(const Node_ n1, const Node_ n2) { return (n1.x == n2.x && n1.y == n2.y); }

int Astar::check_list(const Node_ target_node, std::vector<Node_>& set)
{
    return search_node_from_list(target_node, set);
}

void Astar::swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2)
{
    int idx = search_node_from_list(node, list1);
    if (idx != -1) {
        list2.push_back(list1[idx]);
        list1.erase(list1.begin() + idx);
    }
}

bool Astar::check_obs(const Node_ node)
{
    if (node.x < 0 || node.x >= width_ || node.y < 0 || node.y >= height_) return true;
    return new_map_.data[node.y * width_ + node.x] > 50;
}

void Astar::update_list(const Node_ node)
{
    std::vector<Node_> neighbors;
    create_neighbor_nodes(node, neighbors);
    for (auto& next : neighbors) {
        if (check_obs(next)) continue;
        int o_idx = search_node_from_list(next, open_list_);
        int c_idx = search_node_from_list(next, close_list_);
        if (c_idx != -1) continue;
        if (o_idx == -1) open_list_.push_back(next);
        else if (open_list_[o_idx].f > next.f) open_list_[o_idx] = next;
    }
}

void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>& neighbors)
{
    std::vector<Motion_> motions;
    get_motion(motions);
    for (const auto& m : motions) neighbors.push_back(get_neighbor_node(node, m));
}

void Astar::get_motion(std::vector<Motion_>& list)
{
    list.push_back({1, 0, 1.0}); list.push_back({-1, 0, 1.0});
    list.push_back({0, 1, 1.0}); list.push_back({0, -1, 1.0});
    list.push_back({1, 1, 1.414}); list.push_back({1, -1, 1.414});
    list.push_back({-1, 1, 1.414}); list.push_back({-1, -1, 1.414});
}

Motion_ Astar::motion(const int dx, const int dy, const int cost) { return {dx, dy, static_cast<double>(cost)}; }

Node_ Astar::get_neighbor_node(const Node_ node, const Motion_ motion)
{
    Node_ next;
    next.x = node.x + motion.dx;
    next.y = node.y + motion.dy;
    next.parent_x = node.x;
    next.parent_y = node.y;
    double g = (node.f - make_heuristic(node)) + motion.cost;
    next.f = g + make_heuristic(next);
    return next;
}

std::tuple<int, int> Astar::search_node(const Node_ node)
{
    int idx = search_node_from_list(node, open_list_);
    if (idx != -1) return {1, idx};
    idx = search_node_from_list(node, close_list_);
    if (idx != -1) return {2, idx};
    return {-1, -1};
}

bool Astar::check_parent(const int index, const Node_ node) { return false; }

int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    for (size_t i = 0; i < list.size(); ++i) {
        if (node.x == list[i].x && node.y == list[i].y) return i;
    }
    return -1;
}

void Astar::show_node_point(const Node_ node)
{
    if (!test_show_) return;
    geometry_msgs::msg::PointStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = this->now();
    ps.point.x = node.x * resolution_ + origin_x_;
    ps.point.y = node.y * resolution_ + origin_y_;
    pub_node_point_->publish(ps);

rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_ * 1000)));
}

void Astar::show_path(nav_msgs::msg::Path& current_path)
{
    if (!test_show_) return;
    current_path.header.frame_id = "map";
    pub_current_path_->publish(current_path);
}

void Astar::show_exe_time()
{
    auto duration = this->now().seconds() - begin_.seconds();
    RCLCPP_INFO(this->get_logger(), "Duration = %.2fs", duration);
}

void Astar::planning()
{
    begin_ = this->now();
    for (size_t i = 0; i < way_points_x_.size() - 1; ++i) {
        open_list_.clear(); 
        close_list_.clear();
        start_node_ = set_way_point(i);
        goal_node_ = set_way_point(i+1);
        start_node_.f = make_heuristic(start_node_);
        open_list_.push_back(start_node_);

        while (!open_list_.empty()) {
            Node_ current = select_min_f();
            //show_node_point(current);
            if (check_goal(current)) {
                close_list_.push_back(current);
                create_path(current);
                break;
            }
            swap_node(current, open_list_, close_list_);
            update_list(current);
        }
    }
    show_exe_time();
    RCLCPP_INFO(this->get_logger(), "COMPLETE ASTAR PROGRAM");
}