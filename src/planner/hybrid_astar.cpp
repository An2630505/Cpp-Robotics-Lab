/**
 * Hybrid A* — 运动学约束的连续路径规划
 *
 * 输入:  output/grid.txt (由 map_parser 生成)
 * 输出:  output/hybrid_path.txt (x,y,θ) + output/hybrid_result.ppm
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <cstdlib>

// ========================== 参数 ==========================
const double CELL_SIZE   = 0.2;
const int    GRID_SIZE   = 256;
const double WHEELBASE   = 2.68;
const double MAX_STEER   = 0.6;
const int    NUM_STEER   = 5;
const double ARC_LENGTH  = 1.5;
const int    THETA_BINS  = 72;
const double THETA_RES   = 2.0 * M_PI / THETA_BINS;
const double XY_BIN      = 0.5;  // XY 去重分桶 (m)
const int    XY_BINS     = (int)(GRID_SIZE * CELL_SIZE / XY_BIN) + 1;
const double GOAL_XY_TOL = 2.0;
const double GOAL_TH_TOL = 0.5;

// ========================== 结构 ==========================
struct Pose { double x, y, theta; };
struct HNode {
    double x, y, theta, g, h, f;
    int parent;  // closed_ 索引
    int key() const {
        int ix = std::max(0, std::min(XY_BINS-1, (int)(x / XY_BIN)));
        int iy = std::max(0, std::min(XY_BINS-1, (int)(y / XY_BIN)));
        double th = theta; while (th < 0) th += 2*M_PI; while (th >= 2*M_PI) th -= 2*M_PI;
        int ith = (int)(th / THETA_RES) % THETA_BINS;
        return (iy * XY_BINS + ix) * THETA_BINS + ith;
    }
    bool operator>(const HNode& o) const { return f > o.f; }
};

// ========================== 工具 ==========================
struct GridData { std::vector<std::vector<int>> g; int sr,sc,gr,gc,sz; };
GridData readGrid(const std::string& fp) {
    std::ifstream in(fp); GridData d; std::string line;
    while (std::getline(in, line)) if (line[0]!='#') break;
    std::istringstream iss(line); iss >> d.sz;
    in >> d.sr >> d.sc >> d.gr >> d.gc;
    d.g.assign(d.sz, std::vector<int>(d.sz,0));
    for (int r=0;r<d.sz;r++) for(int c=0;c<d.sz;c++) in>>d.g[r][c];
    return d;
}

// 碰撞检测 (简化版: 矩形 + 格点)
bool collides(const Pose& p, const std::vector<std::vector<int>>& grid) {
    double c=std::cos(p.theta), s=std::sin(p.theta);
    double hw=0.3, fwd=0.3, rev=0.3;  // 近似点, 忽略车身  // 半宽0.9=车1.8, fwd=轴距+前悬=3.18
    double cn[4][2]={{fwd,hw},{fwd,-hw},{-rev,hw},{-rev,-hw}};
    double mx=1e9,Mx=-1e9,my=1e9,My=-1e9;
    for(int i=0;i<4;i++){
        double wx=c*cn[i][0]-s*cn[i][1]+p.x, wy=s*cn[i][0]+c*cn[i][1]+p.y;
        mx=std::min(mx,wx);Mx=std::max(Mx,wx);my=std::min(my,wy);My=std::max(My,wy);
    }
    int cmn=std::max(0,(int)(mx/CELL_SIZE)), cmx=std::min(GRID_SIZE-1,(int)(Mx/CELL_SIZE)+1);
    int rmn=std::max(0,(int)(my/CELL_SIZE)), rmx=std::min(GRID_SIZE-1,(int)(My/CELL_SIZE)+1);
    for(int r=rmn;r<=rmx;r++) for(int ci=cmn;ci<=cmx;ci++){
        if(grid[r][ci]==0) continue;
        double cx=ci*CELL_SIZE+0.1, cy=r*CELL_SIZE+0.1;
        double dx=cx-p.x, dy=cy-p.y;
        double bx=c*dx+s*dy, by=-s*dx+c*dy;
        if(bx>=-rev&&bx<=fwd&&by>=-hw&&by<=hw) return true;
    }
    return false;
}

// 自行车模型积分
Pose step(const Pose& from, double steer, double arc) {
    if(std::abs(steer)<1e-6)
        return {from.x+arc*std::cos(from.theta), from.y+arc*std::sin(from.theta), from.theta};
    double R=WHEELBASE/std::tan(steer), dth=arc/R;
    return {from.x+R*(std::sin(from.theta+dth)-std::sin(from.theta)),
            from.y+R*(std::cos(from.theta)-std::cos(from.theta+dth)),
            from.theta+dth};
}

// 弧段碰撞采样
bool arcCollides(const Pose& from, double steer, double arc,
                 const std::vector<std::vector<int>>& grid) {
    int n=std::max(2,(int)(arc/CELL_SIZE));
    for(int i=0;i<=n;i++)
        if(collides(step(from,steer,arc*i/n), grid)) return true;
    return false;
}

// ========================== Hybrid A* ==========================
std::vector<Pose> hybridAStar(const std::vector<std::vector<int>>& grid,
                               const Pose& start, const Pose& goal) {
    std::cout << "\n=== Hybrid A* ===" << std::endl;
    std::cout << "起点: ("<<start.x<<","<<start.y<<","<<start.theta<<")" << std::endl;
    std::cout << "终点: ("<<goal.x<<","<<goal.y<<","<<goal.theta<<")" << std::endl;
    if(collides(start,grid)){ std::cerr<<"起点碰撞!"<<std::endl; return {}; }
    if(collides(goal,grid)) { std::cerr<<"终点碰撞!"<<std::endl; return {}; }

    // ---- Dijkstra 启发式 (2D, 忽略朝向) ----
    std::vector<std::vector<double>> h2d(GRID_SIZE, std::vector<double>(GRID_SIZE,1e9));
    {
        using Cell=std::pair<double,std::pair<int,int>>;
        std::priority_queue<Cell,std::vector<Cell>,std::greater<Cell>> pq;
        int gr=(int)(goal.y/CELL_SIZE), gc=(int)(goal.x/CELL_SIZE);
        h2d[gr][gc]=0; pq.push({0,{gr,gc}});
        int d8[8][2]={{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
        double c8[8]={1,1,1,1,1.414,1.414,1.414,1.414};
        while(!pq.empty()){
            auto t=pq.top();pq.pop();
            int r=t.second.first, c=t.second.second;
            if(t.first>h2d[r][c]+1e-6) continue;
            for(int d=0;d<8;d++){
                int nr=r+d8[d][0], nc=c+d8[d][1];
                if(nr<0||nr>=GRID_SIZE||nc<0||nc>=GRID_SIZE||grid[nr][nc]==1) continue;
                double nd=t.first+c8[d];
                if(nd<h2d[nr][nc]){h2d[nr][nc]=nd; pq.push({nd,{nr,nc}});}
            }
        }
        std::cout << "启发式预计算完成" << std::endl;
    }

    // ---- 转向角 ----
    std::vector<double> steers;
    for(int i=0;i<NUM_STEER;i++)
        steers.push_back((i-NUM_STEER/2)*MAX_STEER/(NUM_STEER/2));
    std::cout << "转向角: "; for(double s:steers) std::cout<<s<<" "; std::cout<<std::endl;

    // ---- 搜索 ----
    std::priority_queue<HNode,std::vector<HNode>,std::greater<HNode>> open;
    std::unordered_map<int,double> best_g;
    std::vector<HNode> closed;

    HNode sn; sn.x=start.x; sn.y=start.y; sn.theta=start.theta; sn.g=0;
    int sr=(int)(start.y/CELL_SIZE), sc=(int)(start.x/CELL_SIZE);
    sn.h=h2d[sr][sc]*CELL_SIZE; sn.f=sn.h; sn.parent=-1;
    open.push(sn); best_g[sn.key()]=0;

    int expanded=0, iter=0;
    while(!open.empty()){
        HNode cur=open.top(); open.pop(); iter++;
        auto it=best_g.find(cur.key());
        if(it!=best_g.end()&&cur.g>it->second+1e-6) continue;
        expanded++;
        int cur_idx = closed.size();
        closed.push_back(cur);

        // 目标检查
        double dg=std::hypot(cur.x-goal.x, cur.y-goal.y);
        double dth=std::abs(cur.theta-goal.theta);
        while(dth>M_PI) dth=2*M_PI-dth;
        if(dg<GOAL_XY_TOL && dth<GOAL_TH_TOL){
            std::cout<<"找到! 展开:"<<expanded<<" 迭代:"<<iter<<" g:"<<cur.g<<"m"<<std::endl;
            std::vector<Pose> path;
            int idx=closed.size()-1;
            while(idx>=0){path.push_back({closed[idx].x,closed[idx].y,closed[idx].theta}); idx=closed[idx].parent;}
            std::reverse(path.begin(),path.end());
            return path;
        }

        int pidx=cur_idx;
        for(double steer:steers){
            Pose np=step({cur.x,cur.y,cur.theta}, steer, ARC_LENGTH);
            if(np.x<0||np.x>=GRID_SIZE*CELL_SIZE||np.y<0||np.y>=GRID_SIZE*CELL_SIZE) continue;
            if(arcCollides({cur.x,cur.y,cur.theta},steer,ARC_LENGTH,grid)) continue;

            double cost=ARC_LENGTH + std::abs(steer)*ARC_LENGTH*0.3;
            double ng=cur.g+cost;
            HNode child; child.x=np.x; child.y=np.y; child.theta=np.theta;
            child.g=ng; child.parent=pidx;
            int hr=(int)(np.y/CELL_SIZE), hc=(int)(np.x/CELL_SIZE);
            double hv=h2d[hr][hc];
            if(hv>1e8) continue;
            child.h=hv*CELL_SIZE; child.f=child.g+child.h;

            int k=child.key();
            auto it2=best_g.find(k);
            if(it2!=best_g.end()&&ng>=it2->second-1e-6) continue;
            best_g[k]=ng;
            open.push(child);
            // 暂不加入closed, 等展开时再加
        }

        if(iter%10000==0)
            std::cout<<"  迭代"<<iter<<" | open:"<<open.size()<<" | exp:"<<expanded<<std::endl;
    }
    std::cout<<"未找到路径! 展开:"<<expanded<<std::endl;
    return {};
}

// ========================== PPM 输出 ==========================
void savePPM(const std::vector<std::vector<int>>& grid,
             const std::vector<Pose>& path, const Pose& start, const Pose& goal,
             const std::string& fp) {
    std::ofstream out(fp); int S=3, w=GRID_SIZE*S, h=GRID_SIZE*S;
    out<<"P3\n"<<w<<" "<<h<<"\n255\n";
    std::vector<std::vector<char>> ov(GRID_SIZE, std::vector<char>(GRID_SIZE,0));
    for(auto& p:path){int r=(int)(p.y/CELL_SIZE),c=(int)(p.x/CELL_SIZE); if(r>=0&&r<GRID_SIZE&&c>=0&&c<GRID_SIZE) ov[r][c]=1;}
    int sr=(int)(start.y/CELL_SIZE), sc=(int)(start.x/CELL_SIZE);
    int gr=(int)(goal.y/CELL_SIZE),  gc=(int)(goal.x/CELL_SIZE);
    for(int r=0;r<GRID_SIZE;r++) for(int sy=0;sy<S;sy++){
        for(int c=0;c<GRID_SIZE;c++){
            int R,G,B;
            if(r==sr&&c==sc)      {R=0;G=0;B=255;}
            else if(r==gr&&c==gc)  {R=255;G=0;B=0;}
            else if(ov[r][c])      {R=0;G=255;B=0;}
            else if(grid[r][c]==1) {R=30;G=30;B=30;}
            else                   {R=180;G=180;B=180;}
            for(int sx=0;sx<S;sx++) out<<R<<" "<<G<<" "<<B<<" ";
        }
        out<<"\n";
    }
    out.close();
}

// ========================== main ==========================
int main(int argc, char* argv[]) {
    std::string fp="output/grid.txt";
    if(argc>=2) fp=argv[1];

    auto d=readGrid(fp);
    // 膨胀路面: 让窄路变宽, 车身能通过
    auto grid=d.g;
    for(int pass=0;pass<1;pass++){
        auto tmp=grid;
        for(int r=1;r<d.sz-1;r++) for(int c=1;c<d.sz-1;c++)
            if(grid[r][c]==0)
                for(int dr=-1;dr<=1;dr++) for(int dc=-1;dc<=1;dc++)
                    tmp[r+dr][c+dc]=0;
        grid=tmp;
    }
    int fc=0; for(auto&rw:grid) for(int c:rw) if(c==0) fc++;
    std::cout<<"=== Hybrid A* ==="<<std::endl;
    std::cout<<"地图: "<<d.sz<<"×"<<d.sz<<" 膨胀后路面:"<<fc<<std::endl;

    // 起终点: 从 grid.txt 的安全位置出发, 避开边缘
    Pose start={d.sc*CELL_SIZE+0.1, d.sr*CELL_SIZE+0.1, 0};
    Pose goal ={d.gc*CELL_SIZE+0.1, d.gr*CELL_SIZE+0.1, 0};

    // 确保起终点离边缘至少 3m
    start.x=std::max(3.0,std::min(GRID_SIZE*CELL_SIZE-3.0,start.x));
    start.y=std::max(3.0,std::min(GRID_SIZE*CELL_SIZE-3.0,start.y));
    goal.x =std::max(3.0,std::min(GRID_SIZE*CELL_SIZE-3.0,goal.x));
    goal.y =std::max(3.0,std::min(GRID_SIZE*CELL_SIZE-3.0,goal.y));

    // 如果起点碰撞, 在附近搜索安全位姿
    auto findSafe = [&](Pose& p){
        if(!collides(p, grid)) return true;
        for(int rad=1; rad<60; rad++){
            for(int dr=-rad; dr<=rad; dr+=std::max(1,rad/8)){
                for(int dc=-rad; dc<=rad; dc+=std::max(1,rad/8)){
                    int nr=(int)(p.y/CELL_SIZE)+dr, nc=(int)(p.x/CELL_SIZE)+dc;
                    if(nr<5||nr>=GRID_SIZE-5||nc<5||nc>=GRID_SIZE-5) continue;
                    if(nr<10||nr>=GRID_SIZE-10||nc<10||nc>=GRID_SIZE-10) continue;
                    Pose tp={nc*CELL_SIZE+0.1, nr*CELL_SIZE+0.1, 0};
                    // 检查多个朝向
                    bool ok=false;
                    for(double th=0; th<2*M_PI; th+=M_PI/4){
                        tp.theta=th;
                        if(!collides(tp,grid)){ok=true;break;}
                    }
                    if(ok){ p=tp; return true; }
                }
            }
        }
        return false;
    };
    findSafe(start); findSafe(goal);
    // 分别找各自的安全朝向
    double th = std::atan2(goal.y-start.y, goal.x-start.x);
    for(auto* p : {&start, &goal}){
        for(double adj=0; adj<M_PI; adj+=M_PI/8){
            for(int sign=-1; sign<=1; sign+=2){
                double tt = th + sign*adj;
                Pose tp = *p; tp.theta = tt;
                if(!collides(tp, grid)){ p->theta = tt; goto next_pose; }
            }
        }
        next_pose:;
    }

    auto path=hybridAStar(grid, start, goal);
    if(path.empty()){ std::cerr<<"无路径!"<<std::endl; return 1; }

    // 保存
    { std::ofstream out("output/hybrid_path.txt");
      out<<"# Hybrid A* Path\n# steps:"<<path.size()<<"\n# x y theta\n";
      for(auto&p:path) out<<p.x<<" "<<p.y<<" "<<p.theta<<"\n"; }
    savePPM(d.g,path,start,goal,"output/hybrid_result.ppm");  // 原始地图做背景
    std::cout<<"输出: output/hybrid_path.txt, output/hybrid_result.ppm"<<std::endl;

    // 统计
    double len=0; for(size_t i=1;i<path.size();i++) len+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
    double sl=std::hypot(goal.x-start.x,goal.y-start.y);
    std::cout<<"路径:"<<path.size()<<"步 "<<len<<"m 直线:"<<sl<<"m 效率:"<<sl/len*100<<"%"<<std::endl;

    system("open output/hybrid_result.ppm");
    return 0;
}
