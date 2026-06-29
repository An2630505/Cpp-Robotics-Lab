#ifndef PNC_PREDICTION_DYNAMIC_OBSTACLE_H_
#define PNC_PREDICTION_DYNAMIC_OBSTACLE_H_

#include "../common/types.h"

class DynamicObstacle {
public:
    virtual ~DynamicObstacle() = default;
    virtual Vec2d predict(double t) const = 0;
};

class SinusoidalObstacle : public DynamicObstacle {
public:
    SinusoidalObstacle(double x_ref, double y_ref, double heading_ref,
                       double amplitude, double period, double phase = 0.0);
    Vec2d predict(double t) const override;
private:
    double x_ref_, y_ref_, heading_ref_, amplitude_, period_, phase_;
};

#endif  // PNC_PREDICTION_DYNAMIC_OBSTACLE_H_
