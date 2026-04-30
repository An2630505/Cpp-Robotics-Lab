#ifndef ___COMPLEX_PATH_H___
#define ___COMPLEX_PATH_H___

#include "Path.h"
#include <vector>

class ComplexPath : public Path {
public:
    ComplexPath();

    /// 添加路段
    void addStraight(float length);
    void addArc(float length, float radius);  // radius>0=左转, radius<0=右转
    void addSlalom(float length, float A, float omega);

    /// 构建路径（计算各段起始状态）
    void build();

    Eigen::VectorXd getState(float s) override;
    void findNearest(const Eigen::VectorXd &pos, float &s,
                     float &e_y, float &e_psi, float &kappa) override;
    std::string getRefString(float dt, float Vx) const override;

    float totalLength() const { return total_len_; }

private:
    enum SegType { STRAIGHT, ARC, SLALOM };

    struct Segment {
        SegType type;
        float length;
        float param1, param2;  // (R) for ARC; (A, omega) for SLALOM
        float start_s, start_x, start_y, start_psi;
    };

    std::vector<Segment> segments_;
    float total_len_;
    bool built_;
};

#endif
