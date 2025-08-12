#ifndef COSTMAP_GENERATOR_COSTMAP_GENERATOR_NODE_HPP_
#define COSTMAP_GENERATOR_COSTMAP_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "costmap_generator/obstacle_masking.hpp"
#include <memory>
#include <mutex>

namespace costmap_generator {

class CostmapGeneratorNode : public rclcpp::Node {
public:
    CostmapGeneratorNode();

private:
    // Callbacks
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void timer_callback();

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_costmap_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double wall_clearance_distance_;
    double dynamic_obstacle_radius_;
    double publish_frequency_;
    
    // Map data
    nav_msgs::msg::OccupancyGrid::SharedPtr base_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr inflated_map_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr robot_pose_;
    
    // Mutex for thread safety
    std::mutex map_mutex_;
    std::mutex scan_mutex_;
    std::mutex pose_mutex_;
    
    // Helper functions
    void generate_inflated_costmap();
    std::vector<std::pair<std::pair<double, double>, double>> 
        extract_dynamic_obstacles_from_scan();
};

} // namespace costmap_generator

#endif // COSTMAP_GENERATOR_COSTMAP_GENERATOR_NODE_HPP_