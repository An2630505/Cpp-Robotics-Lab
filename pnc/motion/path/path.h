#ifndef PNC_MOTION_PATH_H_
#define PNC_MOTION_PATH_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

/// 多段组合路径：直线 + 圆弧 + S 弯
class Path {
public:
    Path();

    void addStraight(float length);
    void addArc(float length, float radius);
    void addSlalom(float length, float A, float omega);

    void build();

    /// 给定弧长 s，返回 [x, y, psi, kappa]
    Eigen::VectorXd getState(float s);

    /// 找到最近路径点
    void findNearest(const Eigen::VectorXd& pos,
                     float& s, float& e_y, float& e_psi, float& kappa);

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

#endif  // PNC_MOTION_PATH_H_
