#include "costmap_generator/costmap_generator_node.hpp"
#include <chrono>
#include <cmath>

namespace costmap_generator {

CostmapGeneratorNode::CostmapGeneratorNode()
    : Node("costmap_generator_node") {
    
    // Declare parameters
    this->declare_parameter("wall_clearance_distance", 0.3);
    this->declare_parameter("dynamic_obstacle_radius", 0.15);
    this->declare_parameter("publish_frequency", 10.0);
    
    // Get parameters
    wall_clearance_distance_ = this->get_parameter("wall_clearance_distance").as_double();
    dynamic_obstacle_radius_ = this->get_parameter("dynamic_obstacle_radius").as_double();
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();
    
    
    // Create publishers
    inflated_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/inflated_costmap", 10);
    
    // Create subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&CostmapGeneratorNode::map_callback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&CostmapGeneratorNode::scan_callback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&CostmapGeneratorNode::pose_callback, this, std::placeholders::_1));
    
    // Create timer for periodic publishing
    auto timer_period = std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&CostmapGeneratorNode::timer_callback, this));
}

void CostmapGeneratorNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        base_map_ = msg;
        
    }  // ロックを解放してからgenerate_inflated_costmapを呼ぶ
    
    // 初回マップ受信時に即座に膨張マップを生成
    generate_inflated_costmap();
}

void CostmapGeneratorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
}

void CostmapGeneratorNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    robot_pose_ = msg;
}

void CostmapGeneratorNode::timer_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer callback triggered");
    generate_inflated_costmap();
}

void CostmapGeneratorNode::generate_inflated_costmap() {
    std::unique_lock<std::mutex> map_lock(map_mutex_);
    if (!base_map_) {
        RCLCPP_DEBUG(this->get_logger(), "generate_inflated_costmap: base_map_ is null, returning");
        return;
    }
    
    // ベースマップから膨張マップを生成
    
    // 一時的に膨張処理をスキップして、ベースマップをそのまま使用
    int width = base_map_->info.width;
    int height = base_map_->info.height;
    std::vector<std::vector<int>> inflated_grid(height, std::vector<int>(width));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            inflated_grid[y][x] = base_map_->data[index];
        }
    }
    
    // スキャンデータから動的障害物を抽出（一時的にスキップ）
    // std::unique_lock<std::mutex> scan_lock(scan_mutex_);
    // auto dynamic_obstacles = extract_dynamic_obstacles_from_scan();
    // scan_lock.unlock();
    
    // // 動的障害物を含む膨張マップを生成
    // if (!dynamic_obstacles.empty()) {
    //     inflated_grid = ObstacleMasking::create_temporary_grid_with_obstacles(
    //         inflated_grid,
    //         base_map_->info.width, base_map_->info.height, base_map_->info.resolution,
    //         base_map_->info.origin.position.x, base_map_->info.origin.position.y,
    //         wall_clearance_distance_,
    //         dynamic_obstacles);
    // }
    
    // OccupancyGridメッセージを作成
    auto inflated_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    inflated_msg->header = base_map_->header;
    inflated_msg->header.stamp = this->now();
    inflated_msg->info = base_map_->info;
    inflated_msg->data.resize(base_map_->info.width * base_map_->info.height);
    
    // 2D配列を1D配列に変換
    for (size_t y = 0; y < base_map_->info.height; ++y) {
        for (size_t x = 0; x < base_map_->info.width; ++x) {
            size_t index = y * base_map_->info.width + x;
            inflated_msg->data[index] = inflated_grid[y][x];
        }
    }
    
    inflated_map_ = inflated_msg;
    map_lock.unlock();
    
    // パブリッシュ
    inflated_costmap_pub_->publish(*inflated_msg);
}

std::vector<std::pair<std::pair<double, double>, double>> 
CostmapGeneratorNode::extract_dynamic_obstacles_from_scan() {
    std::vector<std::pair<std::pair<double, double>, double>> obstacles;
    
    if (!latest_scan_ || !robot_pose_) {
        return obstacles;
    }
    
    // ロボットの位置と姿勢を取得
    double robot_x = robot_pose_->pose.pose.position.x;
    double robot_y = robot_pose_->pose.pose.position.y;
    
    // クォータニオンからヨー角を計算
    double qx = robot_pose_->pose.pose.orientation.x;
    double qy = robot_pose_->pose.pose.orientation.y;
    double qz = robot_pose_->pose.pose.orientation.z;
    double qw = robot_pose_->pose.pose.orientation.w;
    double robot_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 
                                   1.0 - 2.0 * (qy * qy + qz * qz));
    
    // スキャンデータから障害物を抽出
    double angle = latest_scan_->angle_min;
    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
        double range = latest_scan_->ranges[i];
        
        // 有効な範囲のデータのみ処理
        if (std::isfinite(range) && 
            range >= latest_scan_->range_min && 
            range <= latest_scan_->range_max) {
            
            // ロボット座標系での障害物位置
            double local_x = range * std::cos(angle);
            double local_y = range * std::sin(angle);
            
            // 世界座標系に変換
            double world_x = robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
            double world_y = robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);
            
            // 障害物として追加（半径は固定値を使用）
            obstacles.push_back({{world_x, world_y}, dynamic_obstacle_radius_});
        }
        
        angle += latest_scan_->angle_increment;
    }
    
    return obstacles;
}

} // namespace costmap_generator

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<costmap_generator::CostmapGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}