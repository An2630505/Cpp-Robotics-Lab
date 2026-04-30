#ifndef ___PATH_H___
#define ___PATH_H___

#include <Eigen/Dense>
#include <string>
#include <vector>

/// 多段组合路径：直线 + 圆弧 + S弯
class Path {
public:
    Path();

    /// 添加路段
    void addStraight(float length);
    void addArc(float length, float radius);  // radius>0=左转, radius<0=右转
    void addSlalom(float length, float A, float omega);

    /// 构建路径（计算各段起始状态），添加完所有路段后调用
    void build();

    /// 给定弧长 s，返回 [x, y, psi_ref, kappa]
    Eigen::VectorXd getState(float s);

    /// 给定车辆位置 [x, y, psi]，找到最近路径点
    void findNearest(const Eigen::VectorXd &pos, float &s,
                     float &e_y, float &e_psi, float &kappa);

    /// 生成 MPC 预测时域内 N+1 步的参考轨迹（每行为 [e_y, de_y, e_psi, de_psi]）
    Eigen::MatrixXd getReferenceTrajectory(float s_start, int N, float dt, float Vx);

    /// 返回 REF 头信息字符串（供 Python 画图解析）
    std::string getRefString(float dt, float Vx) const;

    float totalLength() const { return total_len_; }

private:
    enum SegType { STRAIGHT, ARC, SLALOM };

    struct Segment {
        SegType type;
        float length;
        float param1, param2;
        float start_s, start_x, start_y, start_psi;
    };

    std::vector<Segment> segments_;
    float total_len_;
    bool built_;
};

#endif
