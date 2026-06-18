#ifndef PNC_MOTION_MAP_PARSER_H_
#define PNC_MOTION_MAP_PARSER_H_

#include <vector>
#include <string>
#include <cmath>
#include <sys/stat.h>

// ========================== 图像容器 ==========================
struct ImageMap {
    int width, height, max_val;
    std::vector<unsigned char> data;  // row-major, data[y*width + x]

    unsigned char at(int x, int y) const { return data[y * width + x]; }
    unsigned char& at(int x, int y) { return data[y * width + x]; }
    bool inBounds(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
};

// ========================== 路网图 ==========================
struct RoadNode { int id; double x, y; };
struct RoadLane {
    int id, from_node, to_node;
    double width, speed_limit;
    std::vector<double> xs, ys, thetas, kappas;
};
struct RoadGraph {
    std::vector<RoadNode> nodes;
    std::vector<RoadLane> lanes;
};

// ========================== 地图元数据 ==========================
struct MapMeta { double resolution, origin_x, origin_y; };

// ========================== PGM 解析 ==========================
ImageMap readPGM(const std::string& filepath);
MapMeta readYAML(const std::string& filepath);

// ========================== 图像处理 ==========================
std::vector<std::vector<int>> thresholdToGrid(
    const ImageMap& img, int thresh, int out_size);
void dilateGrid(std::vector<std::vector<int>>& grid, int radius);
std::vector<std::vector<int>> skeletonize(
    const std::vector<std::vector<int>>& grid);

// ========================== 骨架 → 路网 ==========================
RoadGraph extractGraph(const std::vector<std::vector<int>>& skeleton,
                       double cell_size);

// ========================== 输出 ==========================
void saveGrid(const std::vector<std::vector<int>>& grid,
              int start_r, int start_c,
              int goal_r, int goal_c, const std::string& path);
void saveGraph(const RoadGraph& g, const std::string& path);
void saveLaneCSVs(const RoadGraph& g, const std::string& dir);
void saveGridPPM(const std::vector<std::vector<int>>& grid,
                 int sr, int sc, int gr, int gc, const std::string& path);

#endif  // PNC_MOTION_MAP_PARSER_H_
