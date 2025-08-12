#ifndef COSTMAP_GENERATOR_COSTMAP_GENERATOR_NODE_HPP_
#define COSTMAP_GENERATOR_COSTMAP_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
    void static_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void dynamic_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void timer_callback();

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr static_obstacles_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacles_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double wall_clearance_distance_;
    double dynamic_obstacle_radius_;
    double publish_frequency_;
    
    // Map data
    nav_msgs::msg::OccupancyGrid::SharedPtr base_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_;
    visualization_msgs::msg::MarkerArray::SharedPtr static_obstacles_;
    visualization_msgs::msg::MarkerArray::SharedPtr dynamic_obstacles_;
    
    // Mutex for thread safety
    std::mutex map_mutex_;
    std::mutex static_obstacles_mutex_;
    std::mutex dynamic_obstacles_mutex_;
    
    // Helper functions
    void generate_global_costmap();
    void generate_local_costmap();
    std::vector<std::pair<std::pair<double, double>, double>> 
        extract_obstacles_from_markers(const visualization_msgs::msg::MarkerArray::SharedPtr& markers);
};

} // namespace costmap_generator

#endif // COSTMAP_GENERATOR_COSTMAP_GENERATOR_NODE_HPP_