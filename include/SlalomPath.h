#ifndef ___SLALOM_PATH_H___
#define ___SLALOM_PATH_H___

#include "Path.h"

class SlalomPath : public Path {
public:
    /// A — 幅值(m)；omega — 角频率(rad/m)；start_x/start_y — 起点
    SlalomPath(float A, float omega, float start_x = 0.0f, float start_y = 0.0f);

    Eigen::VectorXd getState(float s) override;
    void findNearest(const Eigen::VectorXd &pos, float &s,
                     float &e_y, float &e_psi, float &kappa) override;
    std::string getRefString(float dt, float Vx) const override;

private:
    float A_, omega_, start_x_, start_y_;
};

#endif
