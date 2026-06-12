/**
 * 地图解析器 — 图片→高精地图
 *
 * 输入:  PGM (P5 binary) / PPM 图片
 * 输出:  grid.txt + graph.txt + lanes/*.csv (与 scenario.cpp 相同格式)
 *
 * 管线:
 *   PGM → 阈值 → Occupancy Grid → 骨架化 → 交点检测 → 路段追踪 → 图输出
 *
 * 编译:
 *   g++ -std=c++11 -O2 -I./src/planner map_parser.cpp -o ../../build/map_parser
 */

#include "map_parser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <queue>
#include <set>
#include <cstdlib>
#include <sys/stat.h>

// ===================================================================
//  PGM 读取 (P5 binary 格式)
// ===================================================================
//  PGM P5 格式:
//    P5\n
//    width height\n
//    maxval\n
//    <binary pixel data>
// ===================================================================

// 判断文件扩展名
static bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.substr(s.size()-suffix.size()) == suffix;
}

// 用 Python/PIL 将任意图片转为临时 PGM, 返回临时文件路径
static std::string convertToPGM(const std::string& filepath) {
    std::string tmp = "/tmp/map_parser_tmp.pgm";
    // 写临时 Python 脚本避免 shell 转义问题
    std::ofstream py("/tmp/_map_convert.py");
    py << "from PIL import Image\n"
       << "img=Image.open('" << filepath << "').convert('L').resize((256,256))\n"
       << "with open('" << tmp << "','wb') as f:\n"
       << "  f.write(b'P5\\n256 256\\n255\\n')\n"
       << "  f.write(bytes(img.getdata()))\n";
    py.close();
    int ret = system("python3 /tmp/_map_convert.py");
    if (ret != 0) { std::cerr << "图片转换失败!" << std::endl; return ""; }
    return tmp;
}

ImageMap readPGM(const std::string& filepath) {
    // 如果是 PNG/JPG 等, 先用 Python 转 PGM
    std::string actual_path = filepath;
    std::string tmp_pgm;
    if (endsWith(filepath, ".png") || endsWith(filepath, ".jpg") ||
        endsWith(filepath, ".jpeg") || endsWith(filepath, ".bmp")) {
        tmp_pgm = convertToPGM(filepath);
        if (tmp_pgm.empty()) return {};
        actual_path = tmp_pgm;
        std::cout << "已转换 " << filepath << " → PGM" << std::endl;
    }

    std::ifstream in(actual_path, std::ios::binary);
    if (!in) { std::cerr << "无法打开: " << actual_path << std::endl; return {}; }

    ImageMap img;
    std::string magic;
    in >> magic;
    if (magic != "P5") { std::cerr << "不是 P5 PGM!" << std::endl; return {}; }

    // 跳过注释
    char c;
    while ((c = in.peek()) == '#' || c == '\n' || c == '\r') {
        if (c == '#') { std::string dummy; std::getline(in, dummy); }
        else in.get();
    }

    in >> img.width >> img.height >> img.max_val;
    in.get();  // 跳过 maxval 后的单个空白字符

    size_t n = img.width * img.height;
    img.data.resize(n);
    in.read(reinterpret_cast<char*>(img.data.data()), n);

    std::cout << "PGM: " << img.width << "×" << img.height
              << " max=" << img.max_val << std::endl;
    return img;
}

MapMeta readYAML(const std::string& filepath) {
    MapMeta m = {0.05, 0, 0};  // 默认 5cm 分辨率
    std::ifstream in(filepath);
    if (!in) { std::cout << "无 YAML, 使用默认 " << m.resolution << "m/px" << std::endl; return m; }
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("resolution:") != std::string::npos)
            m.resolution = std::stod(line.substr(line.find(':')+1));
        else if (line.find("origin:") != std::string::npos) {
            // origin: [x, y, yaw]
            auto p1 = line.find('['), p2 = line.find(',');
            if (p1 != std::string::npos && p2 != std::string::npos) {
                m.origin_x = std::stod(line.substr(p1+1, p2-p1-1));
                auto p3 = line.find(',', p2+1);
                m.origin_y = std::stod(line.substr(p2+1, p3-p2-1));
            }
        }
    }
    std::cout << "YAML: resolution=" << m.resolution
              << " origin=(" << m.origin_x << "," << m.origin_y << ")" << std::endl;
    return m;
}

// ===================================================================
//  阈值 → Occupancy Grid
// ===================================================================

std::vector<std::vector<int>> thresholdToGrid(const ImageMap& img, int thresh, int out_size) {
    // 计算缩放因子 (原图 → 输出尺寸)
    double sx = (double)img.width  / out_size;
    double sy = (double)img.height / out_size;

    std::vector<std::vector<int>> grid(out_size, std::vector<int>(out_size, 0));
    for (int r = 0; r < out_size; r++) {
        for (int c = 0; c < out_size; c++) {
            // 在原图上采样对应区域
            int x0 = (int)(c * sx), x1 = (int)((c+1) * sx);
            int y0 = (int)(r * sy), y1 = (int)((r+1) * sy);
            int occ = 0, total = 0;
            for (int y = y0; y < y1 && y < img.height; y++)
                for (int x = x0; x < x1 && x < img.width; x++) {
                    if (img.at(x, y) < thresh) occ++;
                    total++;
                }
            // 区域内超过 50% 是暗色 → 障碍
            grid[r][c] = (total > 0 && (double)occ / total > 0.5) ? 1 : 0;
        }
    }
    int free_count = 0;
    for (const auto& row : grid) for (int c : row) if (c == 0) free_count++;
    std::cout << "阈值化: " << out_size << "×" << out_size
              << " free=" << free_count << std::endl;
    return grid;
}

void dilateGrid(std::vector<std::vector<int>>& grid, int radius) {
    int n = grid.size();
    auto tmp = grid;
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++)
            if (grid[r][c] == 1)
                for (int dr = -radius; dr <= radius; dr++)
                    for (int dc = -radius; dc <= radius; dc++) {
                        int nr = r+dr, nc = c+dc;
                        if (nr>=0 && nr<n && nc>=0 && nc<n) tmp[nr][nc] = 1;
                    }
    grid = tmp;
}

// ===================================================================
//  Zhang-Suen 骨架提取
// ===================================================================
//  将 free 区域 (值为0) 细化成单像素宽的骨架线。
//  输入: grid (0=free, 1=obstacle)
//  输出: skeleton (0=骨架线, 1=背景)
// ===================================================================

static int countNeighbors(const std::vector<std::vector<int>>& g, int r, int c, int val) {
    int cnt = 0;
    for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++)
            if (dr || dc) cnt += (g[r+dr][c+dc] == val);
    return cnt;
}

static int transitions(const std::vector<std::vector<int>>& g, int r, int c) {
    // P2, P3, ..., P9, P2 循环的 0→1 跳变次数
    int p[9] = {
        g[r-1][c], g[r-1][c+1], g[r][c+1], g[r+1][c+1],
        g[r+1][c], g[r+1][c-1], g[r][c-1], g[r-1][c-1], g[r-1][c]
    };
    int t = 0;
    for (int i = 0; i < 8; i++) if (p[i]==1 && p[i+1]==0) t++;
    return t;
}

std::vector<std::vector<int>> skeletonize(const std::vector<std::vector<int>>& grid) {
    int n = grid.size();
    // Zhang-Suen 要求: 1=前景(要细化的区域), 0=背景
    // grid: 0=free(路), 1=obstacle
    // → 翻转: skel = 1-free/foreground, 0-obstacle/background
    auto skel = grid;
    for (auto& row : skel) for (int& v : row) v = (v == 0) ? 1 : 0;

    auto countFG = [&](const std::vector<std::vector<int>>& g, int r, int c) {
        int cnt = 0;
        for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++)
                if (dr || dc) cnt += g[r+dr][c+dc];
        return cnt;
    };
    auto trans = [&](const std::vector<std::vector<int>>& g, int r, int c) {
        int p[9] = {g[r-1][c],g[r-1][c+1],g[r][c+1],g[r+1][c+1],
                    g[r+1][c],g[r+1][c-1],g[r][c-1],g[r-1][c-1],g[r-1][c]};
        int t = 0;
        for (int i = 0; i < 8; i++) if (p[i]==0 && p[i+1]==1) t++;
        return t;
    };

    bool changed = true;
    int iter = 0, max_iter = n * 3;
    while (changed && iter < max_iter) {
        changed = false;
        iter++;
        auto prev = skel;

        // 子迭代 1: P2*P4*P6=0 且 P4*P6*P8=0
        for (int r = 1; r < n-1; r++)
            for (int c = 1; c < n-1; c++) {
                if (prev[r][c] != 1) continue;
                int nb = countFG(prev, r, c);
                if (nb < 2 || nb > 6) continue;
                if (trans(prev, r, c) != 1) continue;
                if (prev[r-1][c] && prev[r][c+1] && prev[r+1][c]) continue;
                if (prev[r][c+1] && prev[r+1][c] && prev[r][c-1]) continue;
                skel[r][c] = 0;
                changed = true;
            }
        prev = skel;

        // 子迭代 2: P2*P4*P8=0 且 P2*P6*P8=0
        for (int r = 1; r < n-1; r++)
            for (int c = 1; c < n-1; c++) {
                if (prev[r][c] != 1) continue;
                int nb = countFG(prev, r, c);
                if (nb < 2 || nb > 6) continue;
                if (trans(prev, r, c) != 1) continue;
                if (prev[r-1][c] && prev[r][c+1] && prev[r][c-1]) continue;
                if (prev[r-1][c] && prev[r+1][c] && prev[r][c-1]) continue;
                skel[r][c] = 0;
                changed = true;
            }
    }

    // 翻转回: 0=骨架, 1=背景 (与 grid 格式一致)
    for (auto& row : skel) for (int& v : row) v = 1 - v;
    std::cout << "骨架化: " << iter << " 次迭代" << std::endl;
    return skel;
}

// ===================================================================
//  骨架 → 路网图
// ===================================================================

// 骨架像素分类
enum PixType { BACKGROUND=0, JUNCTION=1, ENDPOINT=2, NORMAL=3 };

static PixType classifyPixel(const std::vector<std::vector<int>>& skel, int r, int c) {
    if (skel[r][c] != 0) return BACKGROUND;
    int nb = 0;
    for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++)
            if (skel[r+dr][c+dc] == 0) nb++;
    nb--;  // 去掉自己
    if (nb >= 3) return JUNCTION;
    if (nb == 1) return ENDPOINT;
    return NORMAL;
}

RoadGraph extractGraph(const std::vector<std::vector<int>>& skel, double cell_size) {
    int n = skel.size();
    RoadGraph g;

    // ---- 步骤 1: 找所有交点和端点 ----
    struct Pt { int r, c; };
    std::vector<Pt> keypoints;
    std::vector<std::vector<int>> key_id(n, std::vector<int>(n, -1));

    for (int r = 1; r < n-1; r++)
        for (int c = 1; c < n-1; c++) {
            auto t = classifyPixel(skel, r, c);
            if (t == JUNCTION || t == ENDPOINT) {
                key_id[r][c] = keypoints.size();
                keypoints.push_back({r, c});
            }
        }

    if (keypoints.size() < 2) {
        std::cout << "骨架无足够关键点 (需要 ≥2)" << std::endl;
        return g;
    }

    // ---- 步骤 2: 从关键点追踪路段 ----
    std::vector<std::vector<bool>> visited(n, std::vector<bool>(n, false));
    int dirs[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};

    for (size_t ki = 0; ki < keypoints.size(); ki++) {
        int sr = keypoints[ki].r, sc = keypoints[ki].c;
        visited[sr][sc] = true;

        for (int d = 0; d < 8; d++) {
            int nr = sr + dirs[d][0], nc = sc + dirs[d][1];
            if (nr<0 || nr>=n || nc<0 || nc>=n) continue;
            if (skel[nr][nc] != 0 || visited[nr][nc]) continue;

            // 追踪一条路段到下一个关键点
            std::vector<Pt> trace;
            trace.push_back({sr, sc});
            int cr = nr, cc = nc;

            while (true) {
                trace.push_back({cr, cc});
                visited[cr][cc] = true;

                auto t = classifyPixel(skel, cr, cc);
                if (t == JUNCTION || t == ENDPOINT) {
                    int end_id = key_id[cr][cc];
                    if (end_id >= 0 && (int)ki < end_id) {  // 避免重复边
                        // 创建路段
                        RoadLane lane;
                        lane.id = g.lanes.size();
                        lane.from_node = ki;
                        lane.to_node = end_id;
                        lane.width = 4.0;
                        lane.speed_limit = 13.9;

                        // 采样中心线 (简化: 直接用像素坐标)
                        for (size_t i = 0; i < trace.size(); i++) {
                            lane.xs.push_back(trace[i].c * cell_size);
                            lane.ys.push_back(trace[i].r * cell_size);
                            if (i+1 < trace.size())
                                lane.thetas.push_back(std::atan2(
                                    trace[i+1].r - trace[i].r,
                                    trace[i+1].c - trace[i].c));
                            else if (!lane.thetas.empty())
                                lane.thetas.push_back(lane.thetas.back());
                            else
                                lane.thetas.push_back(0);
                            lane.kappas.push_back(0);  // 暂不拟合曲率
                        }
                        g.lanes.push_back(lane);
                    }
                    break;
                }
                if (t == NORMAL) {
                    // 找下一个未访问的骨架邻居
                    bool found = false;
                    for (int dd = 0; dd < 8; dd++) {
                        int nnr = cr + dirs[dd][0], nnc = cc + dirs[dd][1];
                        if (nnr<0||nnr>=n||nnc<0||nnc>=n) continue;
                        if (skel[nnr][nnc] == 0 && !visited[nnr][nnc]) {
                            cr = nnr; cc = nnc;
                            found = true; break;
                        }
                    }
                    if (!found) break;  // 死胡同
                } else break;
            }
        }
    }

    // ---- 步骤 3: 创建图节点 ----
    for (size_t i = 0; i < keypoints.size(); i++)
        g.nodes.push_back({(int)i, keypoints[i].c * cell_size, keypoints[i].r * cell_size});

    // 清理未使用节点的路段 (两端都在节点列表中才行)
    std::vector<RoadLane> valid;
    for (auto& l : g.lanes)
        if (l.from_node < (int)g.nodes.size() && l.to_node < (int)g.nodes.size())
            valid.push_back(l);
    g.lanes = valid;

    std::cout << "路网提取: " << g.nodes.size() << " 节点, "
              << g.lanes.size() << " 路段" << std::endl;
    return g;
}

// ===================================================================
//  输出 (与 scenario.cpp 相同接口)
// ===================================================================

void saveGrid(const std::vector<std::vector<int>>& grid, int sr, int sc,
              int gr, int gc, const std::string& path) {
    std::ofstream out(path);
    int n = grid.size();
    out << "# Occupancy Grid (from PGM)\n# size: " << n << "x" << n << "\n";
    out << "# start: (" << sr << ", " << sc << ")\n";
    out << "# goal:  (" << gr << ", " << gc << ")\n";
    out << "# 0=free, 1=obstacle\n" << n << "\n";
    out << sr << " " << sc << "\n" << gr << " " << gc << "\n";
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) out << grid[r][c] << (c<n-1?" ":"");
        out << "\n";
    }
}

void saveGraph(const RoadGraph& g, const std::string& path) {
    std::ofstream out(path);
    out << "# Road Graph (from map parser)\n";
    out << "# NODE id x y\n";
    for (const auto& n : g.nodes)
        out << "NODE " << n.id << " " << n.x << " " << n.y << "\n";
    out << "# LANE id from to length speed_limit file\n";
    for (const auto& l : g.lanes) {
        double len = 0;
        for (size_t i = 1; i < l.xs.size(); i++)
            len += std::hypot(l.xs[i]-l.xs[i-1], l.ys[i]-l.ys[i-1]);
        out << "LANE " << l.id << " " << l.from_node << " " << l.to_node
            << " " << len << " " << l.speed_limit
            << " lanes/lane_" << l.id << ".csv\n";
    }
}

void saveLaneCSVs(const RoadGraph& g, const std::string& dir) {
    mkdir(dir.c_str(), 0755);
    for (const auto& l : g.lanes) {
        std::string fp = dir + "/lane_" + std::to_string(l.id) + ".csv";
        std::ofstream out(fp);
        out << "# Lane " << l.id << "\n# x y theta kappa\n";
        for (size_t i = 0; i < l.xs.size(); i++)
            out << l.xs[i] << " " << l.ys[i] << " "
                << l.thetas[i] << " " << l.kappas[i] << "\n";
    }
}

void saveGridPPM(const std::vector<std::vector<int>>& grid, int sr, int sc,
                 int gr, int gc, const std::string& path) {
    std::ofstream out(path);
    int n = grid.size(), S = 3;
    out << "P3\n" << n*S << " " << n*S << "\n255\n";
    for (int r = 0; r < n; r++)
        for (int sy = 0; sy < S; sy++) {
            for (int c = 0; c < n; c++) {
                int R,G,B;
                if (r==sr && c==sc)      {R=0;G=0;B=255;}
                else if (r==gr && c==gc)  {R=255;G=0;B=0;}
                else if (grid[r][c]==0)   {R=180;G=180;B=180;}
                else                      {R=30;G=30;B=30;}
                for (int sx=0; sx<S; sx++) out<<R<<" "<<G<<" "<<B<<" ";
            }
            out << "\n";
        }
}

// ===================================================================
//  main — 地图解析管线
// ===================================================================

int main(int argc, char* argv[]) {
    std::cout << "=== 地图解析器 ===" << std::endl;

    std::string pgm_path = "map.pgm";
    std::string yaml_path = "map.yaml";
    if (argc >= 2) pgm_path  = argv[1];
    if (argc >= 3) yaml_path = argv[2];

    // 1. 读取 PGM + YAML
    auto img = readPGM(pgm_path);
    if (img.data.empty()) {
        std::cerr << "无法读取 PGM! 用法: map_parser [map.pgm] [map.yaml]" << std::endl;
        std::cerr << "  也可用已有 grid.txt: 可以跳过此步骤" << std::endl;
        return 1;
    }
    auto meta = readYAML(yaml_path);

    // 2. 阈值 → Occupancy Grid (256×256)
    // --invert: 黑=路, 白=障碍 (path3 类型的反色地图)
    bool need_invert = false;
    for (int i = 2; i < argc; i++)
        if (std::string(argv[i]) == "--invert") need_invert = true;

    auto grid = thresholdToGrid(img, img.max_val / 2, 256);
    if (need_invert) {
        for (auto& r : grid) for (int& c : r) c = 1 - c;
        std::cout << "已反转 (黑=路, 白=障碍)" << std::endl;
    }
    dilateGrid(grid, 1);
    std::cout << "网格: " << grid.size() << "×" << grid[0].size() << std::endl;

    // 3. 骨架 → 路网 (可选, 大图可能很慢)
    RoadGraph graph;
    // skeletonize + extractGraph 在大面积自由空间上可能崩溃
    // 离散 A* 可直接用 grid, 不需要图
    std::cout << "跳过骨架提取 (离散 A* 无需路网图)" << std::endl;

    // 5. 选起终点: BFS 找最大连通分量, 再找最远两点
    int sr = 20, sc = 20, gr = 235, gc = 235;
    {
        int n = grid.size();
        std::vector<std::vector<bool>> vis(n, std::vector<bool>(n, false));
        std::vector<std::pair<int,int>> largest;
        int dirs4[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
        for (int r = 0; r < n; r++)
            for (int c = 0; c < n; c++)
                if (grid[r][c] == 0 && !vis[r][c]) {
                    std::vector<std::pair<int,int>> comp;
                    std::queue<std::pair<int,int>> q;
                    q.push({r,c}); vis[r][c] = true;
                    while (!q.empty()) {
                        int cr = q.front().first, cc = q.front().second; q.pop();
                        comp.push_back({cr,cc});
                        for (int d = 0; d < 4; d++) {
                            int nr = cr+dirs4[d][0], nc = cc+dirs4[d][1];
                            if (nr>=0 && nr<n && nc>=0 && nc<n && grid[nr][nc]==0 && !vis[nr][nc])
                                { vis[nr][nc]=true; q.push({nr,nc}); }
                        }
                    }
                    if (comp.size() > largest.size()) largest = comp;
                }
        // 在最大分量里找最远两点
        if (largest.size() >= 2) {
            double max_d = 0;
            int step = std::max(1, (int)largest.size() / 300);
            for (size_t i = 0; i < largest.size(); i += step)
                for (size_t j = i + step; j < largest.size(); j += step) {
                    double d = std::hypot(largest[i].first-largest[j].first,
                                           largest[i].second-largest[j].second);
                    if (d > max_d) {
                        max_d = d;
                        sr = largest[i].first;  sc = largest[i].second;
                        gr = largest[j].first;  gc = largest[j].second;
                    }
                }
        }
    }
    std::cout << "起终点: (" << sr << "," << sc << ") → (" << gr << "," << gc << ")"
              << " (最大连通分量)" << std::endl;

    // 6. 输出
    mkdir("output/lanes", 0755);
    saveGrid(grid, sr, sc, gr, gc, "output/grid.txt");
    saveGridPPM(grid, sr, sc, gr, gc, "output/scenario_grid.ppm");
    if (!graph.nodes.empty()) {
        saveGraph(graph, "output/graph.txt");
        saveLaneCSVs(graph, "output/lanes");
    } else {
        // 生成最小 graph.txt 以便 graph_astar 不报错
        std::ofstream out("output/graph.txt");
        out << "# Road Graph (empty)\n";
    }

    std::cout << "\n完成! 输出:" << std::endl;
    std::cout << "  output/grid.txt" << std::endl;
    std::cout << "  output/scenario_grid.ppm" << std::endl;
    if (!graph.nodes.empty()) {
        std::cout << "  output/graph.txt" << std::endl;
        std::cout << "  output/lanes/*.csv" << std::endl;
    }
    std::cout << "\n后续: ./build/graph_astar && ./build/mpc_planner" << std::endl;
    return 0;
}
