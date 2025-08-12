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
    global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap", 10);
    local_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap", 10);
    
    // Create subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&CostmapGeneratorNode::map_callback, this, std::placeholders::_1));
    
    static_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/static_obstacles", 10,
        std::bind(&CostmapGeneratorNode::static_obstacles_callback, this, std::placeholders::_1));
    
    dynamic_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/dynamic_obstacles", 10,
        std::bind(&CostmapGeneratorNode::dynamic_obstacles_callback, this, std::placeholders::_1));
    
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
    }
    
    // Generate global costmap when map is received
    generate_global_costmap();
}

void CostmapGeneratorNode::static_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(static_obstacles_mutex_);
        static_obstacles_ = msg;
    }
    
    // Generate local costmap when obstacles are updated
    generate_local_costmap();
}

void CostmapGeneratorNode::dynamic_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(dynamic_obstacles_mutex_);
        dynamic_obstacles_ = msg;
    }
    
    // Generate local costmap when obstacles are updated
    generate_local_costmap();
}

void CostmapGeneratorNode::timer_callback() {
    generate_global_costmap();
    generate_local_costmap();
}

void CostmapGeneratorNode::generate_global_costmap() {
    std::unique_lock<std::mutex> lock(map_mutex_);
    if (!base_map_) {
        return;
    }
    
    // Create inflated grid from base map
    auto inflated_grid = ObstacleMasking::create_inflated_grid(
        *base_map_, wall_clearance_distance_, base_map_->info.resolution);
    
    // Create OccupancyGrid message
    auto global_costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    global_costmap_msg->header = base_map_->header;
    global_costmap_msg->header.stamp = this->now();
    global_costmap_msg->info = base_map_->info;
    global_costmap_msg->data.resize(base_map_->info.width * base_map_->info.height);
    
    // Convert 2D array to 1D array
    for (size_t y = 0; y < base_map_->info.height; ++y) {
        for (size_t x = 0; x < base_map_->info.width; ++x) {
            size_t index = y * base_map_->info.width + x;
            global_costmap_msg->data[index] = inflated_grid[y][x];
        }
    }
    
    global_costmap_ = global_costmap_msg;
    lock.unlock();
    
    // Publish global costmap
    global_costmap_pub_->publish(*global_costmap_msg);
}

void CostmapGeneratorNode::generate_local_costmap() {
    std::unique_lock<std::mutex> map_lock(map_mutex_);
    if (!base_map_) {
        return;
    }
    
    // Create empty grid for local costmap
    int width = base_map_->info.width;
    int height = base_map_->info.height;
    std::vector<std::vector<int>> local_grid(height, std::vector<int>(width, 0));
    
    // Extract obstacles from marker arrays
    std::vector<std::pair<std::pair<double, double>, double>> static_obs;
    std::vector<std::pair<std::pair<double, double>, double>> dynamic_obs;
    
    {
        std::lock_guard<std::mutex> static_lock(static_obstacles_mutex_);
        if (static_obstacles_) {
            static_obs = extract_obstacles_from_markers(static_obstacles_);
        }
    }
    
    {
        std::lock_guard<std::mutex> dynamic_lock(dynamic_obstacles_mutex_);
        if (dynamic_obstacles_) {
            dynamic_obs = extract_obstacles_from_markers(dynamic_obstacles_);
        }
    }
    
    // Create local costmap with obstacles
    if (!static_obs.empty() || !dynamic_obs.empty()) {
        local_grid = ObstacleMasking::create_temporary_grid_with_obstacles(
            local_grid,
            base_map_->info.width, base_map_->info.height, base_map_->info.resolution,
            base_map_->info.origin.position.x, base_map_->info.origin.position.y,
            wall_clearance_distance_,
            dynamic_obs, static_obs);
    }
    
    // Create OccupancyGrid message
    auto local_costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    local_costmap_msg->header = base_map_->header;
    local_costmap_msg->header.stamp = this->now();
    local_costmap_msg->info = base_map_->info;
    local_costmap_msg->data.resize(base_map_->info.width * base_map_->info.height);
    
    // Convert 2D array to 1D array
    for (size_t y = 0; y < base_map_->info.height; ++y) {
        for (size_t x = 0; x < base_map_->info.width; ++x) {
            size_t index = y * base_map_->info.width + x;
            local_costmap_msg->data[index] = local_grid[y][x];
        }
    }
    
    map_lock.unlock();
    
    // Publish local costmap
    local_costmap_pub_->publish(*local_costmap_msg);
}

std::vector<std::pair<std::pair<double, double>, double>> 
CostmapGeneratorNode::extract_obstacles_from_markers(const visualization_msgs::msg::MarkerArray::SharedPtr& markers) {
    std::vector<std::pair<std::pair<double, double>, double>> obstacles;
    
    if (!markers) {
        return obstacles;
    }
    
    for (const auto& marker : markers->markers) {
        // Extract position and size from marker
        double x = marker.pose.position.x;
        double y = marker.pose.position.y;
        
        // Use the larger dimension as radius (for spheres, cylinders, etc.)
        double radius = std::max({marker.scale.x, marker.scale.y, marker.scale.z}) / 2.0;
        
        // If radius is too small, use default
        if (radius < 0.01) {
            radius = dynamic_obstacle_radius_;
        }
        
        obstacles.push_back({{x, y}, radius});
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