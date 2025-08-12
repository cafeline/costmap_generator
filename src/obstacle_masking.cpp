#include "costmap_generator/obstacle_masking.hpp"
#include <cmath>
#include <algorithm>

namespace costmap_generator {

std::vector<std::vector<int>> ObstacleMasking::create_inflated_grid(
    const nav_msgs::msg::OccupancyGrid& grid,
    double wall_clearance_distance,
    double resolution) {
    
    int width = grid.info.width;
    int height = grid.info.height;
    
    // 占有格子をコピー
    std::vector<std::vector<int>> inflated_grid(height, std::vector<int>(width));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            inflated_grid[y][x] = grid.data[index];
        }
    }
    
    // 膨張半径をピクセル単位に変換
    int inflation_radius = static_cast<int>(wall_clearance_distance / resolution) + 1;
    
    // グリッドを膨張
    inflate_grid(inflated_grid, width, height, inflation_radius);
    
    return inflated_grid;
}

std::vector<std::vector<int>> ObstacleMasking::create_temporary_grid_with_obstacles(
    const std::vector<std::vector<int>>& base_grid,
    int width, int height, double resolution,
    double origin_x, double origin_y,
    double wall_clearance_distance,
    const std::vector<std::pair<std::pair<double, double>, double>>& dynamic_obstacles,
    const std::vector<std::pair<std::pair<double, double>, double>>& static_obstacles) {
    
    // ベースグリッドをコピー
    auto temp_grid = base_grid;
    
    // 全ての障害物を統合して処理
    std::vector<std::pair<std::pair<double, double>, double>> all_obstacles;
    all_obstacles.insert(all_obstacles.end(), dynamic_obstacles.begin(), dynamic_obstacles.end());
    all_obstacles.insert(all_obstacles.end(), static_obstacles.begin(), static_obstacles.end());
    
    // 各障害物に対して膨張処理
    for (const auto& obstacle : all_obstacles) {
        double obs_x = obstacle.first.first;
        double obs_y = obstacle.first.second;
        double obs_radius = obstacle.second;
        
        // 障害物の位置をグリッド座標に変換
        auto grid_pos = world_to_grid(obs_x, obs_y, origin_x, origin_y, resolution);
        int center_x = grid_pos.first;
        int center_y = grid_pos.second;
        
        // 膨張半径を計算
        double total_radius = obs_radius + wall_clearance_distance;
        int inflation_cells = static_cast<int>(total_radius / resolution) + 1;
        
        // 障害物周囲を膨張
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                // 境界チェック
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    // 実際の距離を計算
                    double dist_x = dx * resolution;
                    double dist_y = dy * resolution;
                    double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);
                    
                    // 障害物半径内なら占有、膨張範囲内なら膨張値
                    if (distance <= obs_radius) {
                        temp_grid[y][x] = INFLATED_VALUE;
                    } else if (distance <= total_radius && temp_grid[y][x] < OCCUPIED_THRESHOLD) {
                        temp_grid[y][x] = INFLATED_VALUE;
                    }
                }
            }
        }
    }
    
    return temp_grid;
}

void ObstacleMasking::inflate_grid(std::vector<std::vector<int>>& grid,
                                  int width, int height,
                                  int inflation_radius) {
    
    // 元の占有セルを記録
    std::vector<std::pair<int, int>> occupied_cells;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (grid[y][x] >= OCCUPIED_THRESHOLD) {
                occupied_cells.emplace_back(x, y);
            }
        }
    }
    
    // 各占有セルの周囲を膨張
    for (const auto& cell : occupied_cells) {
        int center_x = cell.first;
        int center_y = cell.second;
        
        for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
            for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                // 境界チェック
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    // ユークリッド距離で膨張範囲を決定
                    double distance = std::sqrt(dx * dx + dy * dy);
                    if (distance <= inflation_radius) {
                        grid[y][x] = INFLATED_VALUE;
                    }
                }
            }
        }
    }
}

std::pair<int, int> ObstacleMasking::world_to_grid(double world_x, double world_y,
                                                  double origin_x, double origin_y,
                                                  double resolution) {
    int grid_x = static_cast<int>((world_x - origin_x) / resolution);
    int grid_y = static_cast<int>((world_y - origin_y) / resolution);
    return {grid_x, grid_y};
}

} // namespace costmap_generator