#pragma once

#include <vector>
#include <string>
#include <cmath>

// ========================== 图像容器 ==========================
struct ImageMap {
    int width, height;
    int max_val;                // PGM: 255 or 65535
    std::vector<unsigned char> data;  // row-major, data[y*width + x]

    unsigned char at(int x, int y) const { return data[y*width + x]; }
    unsigned char& at(int x, int y)       { return data[y*width + x]; }
    bool inBounds(int x, int y) const { return x>=0 && x<width && y>=0 && y<height; }
};

// ========================== 路网图 (与 scenario.cpp 共用格式) ==========================

struct RoadNode { int id; double x, y; };
struct RoadLane {
    int id, from_node, to_node;
    double width, speed_limit;
    // 中心线采样点 (世界坐标)
    std::vector<double> xs, ys, thetas, kappas;
};

struct RoadGraph {
    std::vector<RoadNode> nodes;
    std::vector<RoadLane> lanes;
};

// ========================== PGM 解析 ==========================

/// 读取 P5 (binary) PGM 文件 (ROS map 标准格式)
ImageMap readPGM(const std::string& filepath);

/// 读取 YAML 元数据 (ROS map_server 格式), 返回 {resolution, origin_x, origin_y}
struct MapMeta { double resolution, origin_x, origin_y; };
MapMeta readYAML(const std::string& filepath);

// ========================== 图像处理 ==========================

/// 阈值化: 灰度 < thresh → occupied(1), >= thresh → free(0)
std::vector<std::vector<int>> thresholdToGrid(const ImageMap& img, int thresh, int out_size);

/// 膨胀 (dilation): 膨胀 occupied 区域
void dilateGrid(std::vector<std::vector<int>>& grid, int radius);

/// Zhang-Suen 骨架提取: free(0) → 细线骨架
std::vector<std::vector<int>> skeletonize(const std::vector<std::vector<int>>& grid);

// ========================== 骨架→路网图 ==========================

/// 从骨架图中提取路网图: 检测交点→追踪路段→拟合几何→输出 RoadGraph
/// @param skeleton  骨架图 (0=free/骨架线, 1=背景)
/// @param cell_size 每格对应的物理尺寸 (m)
RoadGraph extractGraph(const std::vector<std::vector<int>>& skeleton, double cell_size);

// ========================== 输出 ==========================

void saveGrid(const std::vector<std::vector<int>>& grid, int start_r, int start_c,
              int goal_r, int goal_c, const std::string& path);
void saveGraph(const RoadGraph& g, const std::string& path);
void saveLaneCSVs(const RoadGraph& g, const std::string& dir);
void saveGridPPM(const std::vector<std::vector<int>>& grid, int start_r, int start_c,
                 int goal_r, int goal_c, const std::string& path);
