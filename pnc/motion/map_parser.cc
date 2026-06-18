/**
 * 地图解析器 — PGM/YAML → Occupancy Grid + Road Graph
 *
 * 管线: PGM → 阈值 → Occupancy Grid → 骨架化 → 交点检测 → 路段追踪 → 图输出
 */
#include "map_parser.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <queue>
#include <set>
#include <cstdlib>

// ===================================================================
//  PGM 读取 (P5 binary 格式)
// ===================================================================

static bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size()
        && s.substr(s.size() - suffix.size()) == suffix;
}

static std::string convertToPGM(const std::string& filepath) {
    std::string tmp = "/tmp/map_parser_tmp.pgm";
    std::ofstream py("/tmp/_map_convert.py");
    py << "from PIL import Image\n"
       << "img=Image.open('" << filepath << "').convert('L').resize((256,256))\n"
       << "with open('" << tmp << "','wb') as f:\n"
       << "  f.write(b'P5\\n256 256\\n255\\n')\n"
       << "  f.write(bytes(img.getdata()))\n";
    py.close();
    int ret = system("python3 /tmp/_map_convert.py");
    if (ret != 0) return "";
    return tmp;
}

ImageMap readPGM(const std::string& filepath) {
    std::string actual_path = filepath;
    std::string tmp_pgm;
    if (endsWith(filepath, ".png") || endsWith(filepath, ".jpg") ||
        endsWith(filepath, ".jpeg") || endsWith(filepath, ".bmp")) {
        tmp_pgm = convertToPGM(filepath);
        if (tmp_pgm.empty()) return {};
        actual_path = tmp_pgm;
    }

    std::ifstream in(actual_path, std::ios::binary);
    if (!in) return {};

    ImageMap img;
    std::string magic;
    in >> magic;
    if (magic != "P5") return {};

    char c;
    while ((c = in.peek()) == '#' || c == '\n' || c == '\r') {
        if (c == '#') { std::string dummy; std::getline(in, dummy); }
        else in.get();
    }

    in >> img.width >> img.height >> img.max_val;
    in.get();

    size_t n = img.width * img.height;
    img.data.resize(n);
    in.read(reinterpret_cast<char*>(img.data.data()), n);
    return img;
}

MapMeta readYAML(const std::string& filepath) {
    MapMeta m = {0.05, 0, 0};
    std::ifstream in(filepath);
    if (!in) return m;
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("resolution:") != std::string::npos)
            m.resolution = std::stod(line.substr(line.find(':') + 1));
        else if (line.find("origin:") != std::string::npos) {
            auto p1 = line.find('['), p2 = line.find(',');
            if (p1 != std::string::npos && p2 != std::string::npos) {
                m.origin_x = std::stod(line.substr(p1 + 1, p2 - p1 - 1));
                auto p3 = line.find(',', p2 + 1);
                m.origin_y = std::stod(line.substr(p2 + 1, p3 - p2 - 1));
            }
        }
    }
    return m;
}

// ===================================================================
//  阈值 → Occupancy Grid
// ===================================================================

std::vector<std::vector<int>> thresholdToGrid(
    const ImageMap& img, int thresh, int out_size) {
    double sx = (double)img.width / out_size;
    double sy = (double)img.height / out_size;
    std::vector<std::vector<int>> grid(out_size,
        std::vector<int>(out_size, 0));
    for (int r = 0; r < out_size; r++)
        for (int c = 0; c < out_size; c++) {
            int x0 = (int)(c * sx), x1 = (int)((c + 1) * sx);
            int y0 = (int)(r * sy), y1 = (int)((r + 1) * sy);
            int occ = 0, total = 0;
            for (int y = y0; y < y1 && y < img.height; y++)
                for (int x = x0; x < x1 && x < img.width; x++) {
                    if (img.at(x, y) < thresh) occ++;
                    total++;
                }
            grid[r][c] = (total > 0 && (double)occ / total > 0.5) ? 1 : 0;
        }
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
                        int nr = r + dr, nc = c + dc;
                        if (nr >= 0 && nr < n && nc >= 0 && nc < n)
                            tmp[nr][nc] = 1;
                    }
    grid = tmp;
}

// ===================================================================
//  Zhang-Suen 骨架提取
// ===================================================================

std::vector<std::vector<int>> skeletonize(
    const std::vector<std::vector<int>>& grid) {
    int n = grid.size();
    auto skel = grid;
    for (auto& row : skel) for (int& v : row) v = (v == 0) ? 1 : 0;

    auto countFG = [&](const std::vector<std::vector<int>>& g,
                        int r, int c) {
        int cnt = 0;
        for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++)
                if (dr || dc) cnt += g[r + dr][c + dc];
        return cnt;
    };
    auto trans = [&](const std::vector<std::vector<int>>& g,
                      int r, int c) {
        int p[9] = {g[r-1][c],g[r-1][c+1],g[r][c+1],g[r+1][c+1],
                    g[r+1][c],g[r+1][c-1],g[r][c-1],g[r-1][c-1],g[r-1][c]};
        int t = 0;
        for (int i = 0; i < 8; i++) if (p[i] == 0 && p[i+1] == 1) t++;
        return t;
    };

    bool changed = true;
    int iter = 0, max_iter = n * 3;
    while (changed && iter < max_iter) {
        changed = false; iter++;
        auto prev = skel;
        for (int r = 1; r < n - 1; r++)
            for (int c = 1; c < n - 1; c++) {
                if (prev[r][c] != 1) continue;
                int nb = countFG(prev, r, c);
                if (nb < 2 || nb > 6) continue;
                if (trans(prev, r, c) != 1) continue;
                if (prev[r-1][c] && prev[r][c+1] && prev[r+1][c]) continue;
                if (prev[r][c+1] && prev[r+1][c] && prev[r][c-1]) continue;
                skel[r][c] = 0; changed = true;
            }
        prev = skel;
        for (int r = 1; r < n - 1; r++)
            for (int c = 1; c < n - 1; c++) {
                if (prev[r][c] != 1) continue;
                int nb = countFG(prev, r, c);
                if (nb < 2 || nb > 6) continue;
                if (trans(prev, r, c) != 1) continue;
                if (prev[r-1][c] && prev[r][c+1] && prev[r][c-1]) continue;
                if (prev[r-1][c] && prev[r+1][c] && prev[r][c-1]) continue;
                skel[r][c] = 0; changed = true;
            }
    }
    for (auto& row : skel) for (int& v : row) v = 1 - v;
    return skel;
}

// ===================================================================
//  骨架 → 路网图
// ===================================================================

enum PixType { BKGD = 0, JUNCTION = 1, ENDPOINT = 2, NORMAL = 3 };

static PixType classifyPixel(const std::vector<std::vector<int>>& skel,
                              int r, int c) {
    if (skel[r][c] != 0) return BKGD;
    int nb = 0;
    for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++)
            if (skel[r + dr][c + dc] == 0) nb++;
    nb--;
    if (nb >= 3) return JUNCTION;
    if (nb == 1) return ENDPOINT;
    return NORMAL;
}

RoadGraph extractGraph(const std::vector<std::vector<int>>& skel,
                       double cell_size) {
    int n = skel.size();
    RoadGraph g;

    struct Pt { int r, c; };
    std::vector<Pt> keypoints;
    std::vector<std::vector<int>> key_id(n, std::vector<int>(n, -1));

    for (int r = 1; r < n - 1; r++)
        for (int c = 1; c < n - 1; c++) {
            auto t = classifyPixel(skel, r, c);
            if (t == JUNCTION || t == ENDPOINT) {
                key_id[r][c] = keypoints.size();
                keypoints.push_back({r, c});
            }
        }

    if (keypoints.size() < 2) return g;

    std::vector<std::vector<bool>> visited(n, std::vector<bool>(n, false));
    int dirs[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};

    for (size_t ki = 0; ki < keypoints.size(); ki++) {
        int sr = keypoints[ki].r, sc = keypoints[ki].c;
        visited[sr][sc] = true;

        for (int d = 0; d < 8; d++) {
            int nr = sr + dirs[d][0], nc = sc + dirs[d][1];
            if (nr < 0 || nr >= n || nc < 0 || nc >= n) continue;
            if (skel[nr][nc] != 0 || visited[nr][nc]) continue;

            std::vector<Pt> trace;
            trace.push_back({sr, sc});
            int cr = nr, cc = nc;

            while (true) {
                trace.push_back({cr, cc});
                visited[cr][cc] = true;

                auto t = classifyPixel(skel, cr, cc);
                if (t == JUNCTION || t == ENDPOINT) {
                    int end_id = key_id[cr][cc];
                    if (end_id >= 0 && (int)ki < end_id) {
                        RoadLane lane;
                        lane.id = g.lanes.size();
                        lane.from_node = ki;
                        lane.to_node = end_id;
                        lane.width = 4.0;
                        lane.speed_limit = 13.9;
                        for (size_t i = 0; i < trace.size(); i++) {
                            lane.xs.push_back(trace[i].c * cell_size);
                            lane.ys.push_back(trace[i].r * cell_size);
                            if (i + 1 < trace.size())
                                lane.thetas.push_back(std::atan2(
                                    trace[i+1].r - trace[i].r,
                                    trace[i+1].c - trace[i].c));
                            else if (!lane.thetas.empty())
                                lane.thetas.push_back(lane.thetas.back());
                            else
                                lane.thetas.push_back(0);
                            lane.kappas.push_back(0);
                        }
                        g.lanes.push_back(lane);
                    }
                    break;
                }
                if (t == NORMAL) {
                    bool found = false;
                    for (int dd = 0; dd < 8; dd++) {
                        int nnr = cr + dirs[dd][0], nnc = cc + dirs[dd][1];
                        if (nnr < 0 || nnr >= n || nnc < 0 || nnc >= n) continue;
                        if (skel[nnr][nnc] == 0 && !visited[nnr][nnc]) {
                            cr = nnr; cc = nnc; found = true; break;
                        }
                    }
                    if (!found) break;
                } else break;
            }
        }
    }

    for (size_t i = 0; i < keypoints.size(); i++)
        g.nodes.push_back({(int)i, keypoints[i].c * cell_size,
                           keypoints[i].r * cell_size});

    std::vector<RoadLane> valid;
    for (auto& l : g.lanes)
        if (l.from_node < (int)g.nodes.size()
            && l.to_node < (int)g.nodes.size())
            valid.push_back(l);
    g.lanes = valid;
    return g;
}

// ===================================================================
//  输出
// ===================================================================

void saveGrid(const std::vector<std::vector<int>>& grid,
              int sr, int sc, int gr, int gc, const std::string& path) {
    std::ofstream out(path);
    int n = grid.size();
    out << "# Occupancy Grid\n# size: " << n << "x" << n << "\n"
        << "# start: (" << sr << ", " << sc << ")\n"
        << "# goal:  (" << gr << ", " << gc << ")\n"
        << "# 0=free, 1=obstacle\n" << n << "\n"
        << sr << " " << sc << "\n" << gr << " " << gc << "\n";
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++)
            out << grid[r][c] << (c < n - 1 ? " " : "");
        out << "\n";
    }
}

void saveGraph(const RoadGraph& g, const std::string& path) {
    std::ofstream out(path);
    out << "# Road Graph\n";
    out << "# NODE id x y\n";
    for (const auto& n : g.nodes)
        out << "NODE " << n.id << " " << n.x << " " << n.y << "\n";
    out << "# LANE id from to length speed_limit file\n";
    for (const auto& l : g.lanes) {
        double len = 0;
        for (size_t i = 1; i < l.xs.size(); i++)
            len += std::hypot(l.xs[i] - l.xs[i-1], l.ys[i] - l.ys[i-1]);
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

void saveGridPPM(const std::vector<std::vector<int>>& grid,
                 int sr, int sc, int gr, int gc, const std::string& path) {
    std::ofstream out(path);
    int n = grid.size(), S = 3;
    out << "P3\n" << n * S << " " << n * S << "\n255\n";
    for (int r = 0; r < n; r++)
        for (int sy = 0; sy < S; sy++) {
            for (int c = 0; c < n; c++) {
                int R, G, B;
                if (r == sr && c == sc)      { R = 0; G = 0; B = 255; }
                else if (r == gr && c == gc)  { R = 255; G = 0; B = 0; }
                else if (grid[r][c] == 0)     { R = 180; G = 180; B = 180; }
                else                          { R = 30; G = 30; B = 30; }
                for (int sx = 0; sx < S; sx++)
                    out << R << " " << G << " " << B << " ";
            }
            out << "\n";
        }
}
