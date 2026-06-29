#include "dynamic_obstacle.h"
#include <cmath>

SinusoidalObstacle::SinusoidalObstacle(
    double x_ref, double y_ref, double heading_ref,
    double amplitude, double period, double phase)
    : x_ref_(x_ref), y_ref_(y_ref), heading_ref_(heading_ref),
      amplitude_(amplitude), period_(period), phase_(phase) {}

Vec2d SinusoidalObstacle::predict(double t) const {
    double lat = amplitude_ * std::sin(2.0 * M_PI * t / period_ + phase_);
    Vec2d pos;
    pos.x = x_ref_ - lat * std::sin(heading_ref_);
    pos.y = y_ref_ + lat * std::cos(heading_ref_);
    return pos;
}
