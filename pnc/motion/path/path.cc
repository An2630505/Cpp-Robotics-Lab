#include "path.h"

#include <cmath>
#include <sstream>

Path::Path() : total_len_(0.0f), built_(false) {}

void Path::addStraight(float length) {
    segments_.push_back({STRAIGHT, length, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
}

void Path::addArc(float length, float radius) {
    segments_.push_back({ARC, length, radius, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
}

void Path::addSlalom(float length, float A, float omega) {
    segments_.push_back({SLALOM, length, A, omega, 0.0f, 0.0f, 0.0f, 0.0f});
}

void Path::build() {
    float s = 0.0f, x = 0.0f, y = 0.0f, psi = 0.0f;

    for (auto& seg : segments_) {
        seg.start_s = s;
        seg.start_x = x;
        seg.start_y = y;
        seg.start_psi = psi;

        float L = seg.length;

        switch (seg.type) {
        case STRAIGHT:
            x += L * std::cos(psi);
            y += L * std::sin(psi);
            break;

        case ARC: {
            float R = seg.param1;
            float da = L / R;
            float psi_end = psi + da;
            x += R * (std::sin(psi_end) - std::sin(psi));
            y -= R * (std::cos(psi_end) - std::cos(psi));
            psi = psi_end;
            break;
        }

        case SLALOM: {
            float A = seg.param1, omega = seg.param2;
            float y_local_end = A * std::sin(omega * L);
            float dy_end = A * omega * std::cos(omega * L);
            x += L * std::cos(psi) - y_local_end * std::sin(psi);
            y += L * std::sin(psi) + y_local_end * std::cos(psi);
            psi += std::atan2(dy_end, 1.0f);
            break;
        }
        }
        s += L;
    }
    total_len_ = s;
    built_ = true;
}

Eigen::VectorXd Path::getState(float s) {
    Eigen::VectorXd state(4);
    if (!built_ || segments_.empty()) {
        state << s, 0.0f, 0.0f, 0.0f;
        return state;
    }

    if (s < 0.0f) s = 0.0f;
    if (s > total_len_) s = total_len_;

    size_t idx = 0;
    while (idx + 1 < segments_.size() && segments_[idx + 1].start_s <= s)
        idx++;

    const auto& seg = segments_[idx];
    float sl = s - seg.start_s;
    float x0 = seg.start_x, y0 = seg.start_y, psi0 = seg.start_psi;

    switch (seg.type) {
    case STRAIGHT:
        state << x0 + sl * std::cos(psi0),
                 y0 + sl * std::sin(psi0),
                 psi0,
                 0.0f;
        break;

    case ARC: {
        float R = seg.param1;
        float da = sl / R;
        float ps = psi0 + da;
        state << x0 + R * (std::sin(ps) - std::sin(psi0)),
                 y0 - R * (std::cos(ps) - std::cos(psi0)),
                 ps,
                 1.0f / R;
        break;
    }

    case SLALOM: {
        float A = seg.param1, omega = seg.param2;
        float y_local = A * std::sin(omega * sl);
        float dy = A * omega * std::cos(omega * sl);
        float ddy = -A * omega * omega * std::sin(omega * sl);
        float ps = psi0 + std::atan2(dy, 1.0f);
        float denom = std::pow(1.0f + dy * dy, 1.5f);
        float kappa = (denom > 1e-6f) ? ddy / denom : 0.0f;
        state << x0 + sl * std::cos(psi0) - y_local * std::sin(psi0),
                 y0 + sl * std::sin(psi0) + y_local * std::cos(psi0),
                 ps,
                 kappa;
        break;
    }
    }
    return state;
}

void Path::findNearest(const Eigen::VectorXd& pos,
                        float& s, float& e_y, float& e_psi, float& kappa) {
    if (!built_ || total_len_ <= 0.0f) {
        s = 0.0f; e_y = 0.0f; e_psi = 0.0f; kappa = 0.0f;
        return;
    }

    float best_s = 0.0f;
    float best_dist = 1e9f;
    float step = 0.5f;
    int ns = static_cast<int>(total_len_ / step);

    for (int i = 0; i <= ns; i++) {
        float ss = i * step;
        if (ss > total_len_) ss = total_len_;
        Eigen::VectorXd st = getState(ss);
        float dx = pos(0) - st(0);
        float dy = pos(1) - st(1);
        float d = dx * dx + dy * dy;
        if (d < best_dist) { best_dist = d; best_s = ss; }
    }

    for (int iter = 0; iter < 5; iter++) {
        float ds = 0.1f;
        Eigen::VectorXd st1 = getState(std::min(best_s + ds, total_len_));
        float d_plus = std::pow(pos(0) - st1(0), 2) + std::pow(pos(1) - st1(1), 2);
        if (d_plus < best_dist) {
            best_s = std::min(best_s + ds, total_len_);
            best_dist = d_plus;
            continue;
        }
        Eigen::VectorXd stm1 = getState(std::max(best_s - ds, 0.0f));
        float d_minus = std::pow(pos(0) - stm1(0), 2) + std::pow(pos(1) - stm1(1), 2);
        if (d_minus < best_dist) {
            best_s = std::max(best_s - ds, 0.0f);
            best_dist = d_minus;
        } else {
            break;
        }
    }

    s = best_s;
    Eigen::VectorXd st = getState(s);
    float px = st(0), py = st(1), ppsi = st(2);
    kappa = st(3);

    float dx = pos(0) - px;
    float dy = pos(1) - py;
    e_y = dx * (-std::sin(ppsi)) + dy * std::cos(ppsi);

    e_psi = pos(2) - ppsi;
    e_psi = std::fmod(e_psi + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (e_psi < 0) e_psi += 2.0f * static_cast<float>(M_PI);
    e_psi -= static_cast<float>(M_PI);
}

std::string Path::getRefString(float dt, float Vx) const {
    std::ostringstream oss;
    oss << "type=complex dt=" << dt << " Vx=" << Vx << " segments";
    for (const auto& seg : segments_) {
        switch (seg.type) {
        case STRAIGHT: oss << "|S:" << seg.length; break;
        case ARC:      oss << "|A:" << seg.length << ":" << seg.param1; break;
        case SLALOM:   oss << "|Z:" << seg.length << ":" << seg.param1 << ":" << seg.param2; break;
        }
    }
    return oss.str();
}
