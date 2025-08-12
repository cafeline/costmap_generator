#ifndef COSTMAP_GENERATOR_OBSTACLE_MASKING_HPP_
#define COSTMAP_GENERATOR_OBSTACLE_MASKING_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <utility>

namespace costmap_generator {

/**
 * @brief 障害物領域をマスクするユーティリティクラス
 */
class ObstacleMasking {
public:
    /**
     * @brief 壁からの距離を考慮した膨張マップを作成
     * @param grid 元の占有格子マップ
     * @param wall_clearance_distance 壁からの最小距離 [m]
     * @param resolution グリッドの解像度 [m/cell]
     * @return 膨張された占有格子マップ
     */
    static std::vector<std::vector<int>> create_inflated_grid(
        const nav_msgs::msg::OccupancyGrid& grid,
        double wall_clearance_distance,
        double resolution);

    /**
     * @brief 既存のグリッドに障害物を追加して膨張マップを作成
     * @param base_grid ベースとなる占有格子マップ（2D配列）
     * @param width グリッドの幅
     * @param height グリッドの高さ
     * @param resolution グリッドの解像度 [m/cell]
     * @param origin_x グリッドの原点X座標
     * @param origin_y グリッドの原点Y座標
     * @param wall_clearance_distance 壁からの最小距離 [m]
     * @param dynamic_obstacles 動的障害物情報（位置と半径のペア）
     * @param static_obstacles 静的障害物情報（位置と半径のペア）
     * @return 障害物を反映した膨張占有格子マップ
     */
    static std::vector<std::vector<int>> create_temporary_grid_with_obstacles(
        const std::vector<std::vector<int>>& base_grid,
        int width, int height, double resolution,
        double origin_x, double origin_y,
        double wall_clearance_distance,
        const std::vector<std::pair<std::pair<double, double>, double>>& dynamic_obstacles,
        const std::vector<std::pair<std::pair<double, double>, double>>& static_obstacles = {});

    /**
     * @brief グリッドを膨張させる
     * @param grid 元のグリッド（変更される）
     * @param width グリッドの幅
     * @param height グリッドの高さ
     * @param inflation_radius 膨張半径（ピクセル単位）
     */
    static void inflate_grid(std::vector<std::vector<int>>& grid,
                            int width, int height,
                            int inflation_radius);

    // 閾値定数
    static constexpr int OCCUPIED_THRESHOLD = 65;
    static constexpr int FREE_THRESHOLD = 25;
    static constexpr int INFLATED_VALUE = 100;

private:
    /**
     * @brief 世界座標をグリッド座標に変換
     * @param world_x 世界座標X
     * @param world_y 世界座標Y
     * @param origin_x 原点X座標
     * @param origin_y 原点Y座標
     * @param resolution 解像度
     * @return グリッド座標のペア(x, y)
     */
    static std::pair<int, int> world_to_grid(double world_x, double world_y,
                                            double origin_x, double origin_y,
                                            double resolution);
};

} // namespace costmap_generator

#endif // COSTMAP_GENERATOR_OBSTACLE_MASKING_HPP_