#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>

#include "control/mpc/mpc.h"
#include "control/kf/kf.h"
#include "control/pid/pid.h"
#include "control/lqr/lqr.h"
#include "motion/bicycle_model/bicycle_model.h"
#include "motion/path/path.h"
#include "motion/astar/astar.h"
#include "motion/hybrid_astar/hybrid_astar.h"
#include "motion/mpc_planner/mpc_planner.h"
#include "motion/map_parser/map_parser.h"

namespace py = pybind11;

PYBIND11_MODULE(pnc, m) {
    m.doc() = "PNC Robotics Lab — C++ algorithm library";

    // ======================== Control ========================

    // ---- MPC ----
    py::class_<MPC>(m, "MPC")
        .def(py::init<>())
        .def("init", &MPC::Init,
             py::arg("A"), py::arg("B"), py::arg("C"),
             py::arg("Q"), py::arg("R"), py::arg("S"), py::arg("N"))
        .def("predict", &MPC::predict, py::arg("y_ref"), py::arg("x_obs"));

    // ---- KF ----
    py::class_<KF>(m, "KF")
        .def(py::init<>())
        .def_readwrite("x_hat", &KF::x_hat)
        .def_readwrite("x_post", &KF::x_post)
        .def_readwrite("y_hat", &KF::y_hat)
        .def_readwrite("y_meas", &KF::y_meas)
        .def_readwrite("y_post", &KF::y_post)
        .def("init", &KF::init,
             py::arg("A"), py::arg("B"), py::arg("C"),
             py::arg("P"), py::arg("Q"), py::arg("R"), py::arg("x0"))
        .def("predict", &KF::predict, py::arg("u"))
        .def("correct", &KF::correct, py::arg("measurement"))
        .def("update", &KF::update,
             py::arg("measurement"),
             py::arg("u") = Eigen::VectorXd::Zero(2));

    // ---- PID ----
    py::class_<PID>(m, "PID")
        .def(py::init<int>(), py::arg("n"))
        .def(py::init<int, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>(),
             py::arg("n"), py::arg("kp"), py::arg("ki"), py::arg("kd"))
        .def("init", &PID::init)
        .def("set_param", &PID::setParam,
             py::arg("kp"), py::arg("ki"), py::arg("kd"),
             py::arg("min_out"), py::arg("max_out"))
        .def("position_pid", &PID::positionPID,
             py::arg("target"), py::arg("current"))
        .def("incremental_pid", &PID::incrementalPID,
             py::arg("target"), py::arg("current"));

    // ---- LQR ----
    py::class_<LQR>(m, "LQR")
        .def(py::init<>())
        .def("init", &LQR::Init,
             py::arg("A"), py::arg("B"), py::arg("C"),
             py::arg("Q"), py::arg("R"), py::arg("S"))
        .def("run", &LQR::run, py::arg("y_ref"), py::arg("x_obs"));

    // ======================== Motion ========================

    // ---- BicycleModel ----
    py::class_<BicycleModel>(m, "BicycleModel")
        .def(py::init<>())
        .def(py::init<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd,
                      Eigen::MatrixXd, Eigen::MatrixXd>(),
             py::arg("A"), py::arg("B1"), py::arg("B2"),
             py::arg("C"), py::arg("D"))
        .def_readwrite("x", &BicycleModel::x)
        .def_readwrite("y", &BicycleModel::y)
        .def_readonly("nx", &BicycleModel::nx)
        .def_readonly("ny", &BicycleModel::ny)
        .def_readonly("nu", &BicycleModel::nu)
        .def_readwrite("kf", &BicycleModel::kf)
        .def("init", &BicycleModel::Init,
             py::arg("x0"), py::arg("u0") = Eigen::VectorXd::Zero(1))
        .def("step", &BicycleModel::step,
             py::arg("dt"), py::arg("w"),
             py::arg("u") = Eigen::VectorXd::Zero(1))
        .def("get_time", &BicycleModel::getTime);

    // ---- Path ----
    py::class_<Path>(m, "Path")
        .def(py::init<>())
        .def("add_straight", &Path::addStraight, py::arg("length"))
        .def("add_arc", &Path::addArc, py::arg("length"), py::arg("radius"))
        .def("add_slalom", &Path::addSlalom,
             py::arg("length"), py::arg("A"), py::arg("omega"))
        .def("build", &Path::build)
        .def("get_state", &Path::getState, py::arg("s"))
        .def("find_nearest", [](Path& self, const Eigen::VectorXd& pos) {
            float s, e_y, e_psi, kappa;
            self.findNearest(pos, s, e_y, e_psi, kappa);
            return py::make_tuple(s, e_y, e_psi, kappa);
        }, py::arg("pos"))
        .def("get_ref_string", &Path::getRefString,
             py::arg("dt"), py::arg("Vx"))
        .def("total_length", &Path::totalLength);

    // ---- 基础类型 (需在其他 Motion 类之前注册) ----
    py::class_<Point>(m, "Point")
        .def(py::init<>())
        .def_readwrite("row", &Point::row)
        .def_readwrite("col", &Point::col);

    py::class_<Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("x", &Pose::x)
        .def_readwrite("y", &Pose::y)
        .def_readwrite("theta", &Pose::theta);

    // ---- A* (wrapper: 用 row/col 替代 Point 避免 segfault) ----
    py::class_<AStar>(m, "AStar")
        .def(py::init([](const std::vector<std::vector<int>>& grid,
                         int sr, int sc, int gr, int gc) {
            return std::make_unique<AStar>(grid, Point{sr, sc}, Point{gr, gc});
        }), py::arg("grid"), py::arg("sr"), py::arg("sc"),
            py::arg("gr"), py::arg("gc"))
        .def("find_path", &AStar::findPath)
        .def("get_search_history", &AStar::getSearchHistory);

    // ---- HybridAStar ----
    py::class_<HybridAStar>(m, "HybridAStar")
        .def(py::init<const std::vector<std::vector<int>>&>(),
             py::arg("grid"))
        .def("set_wheelbase", &HybridAStar::setWheelbase)
        .def("set_max_steer", &HybridAStar::setMaxSteer)
        .def("set_num_steer", &HybridAStar::setNumSteer)
        .def("set_arc_length", &HybridAStar::setArcLength)
        .def("set_cell_size", &HybridAStar::setCellSize)
        .def("set_goal_xy_tol", &HybridAStar::setGoalXYTol)
        .def("set_goal_th_tol", &HybridAStar::setGoalThTol)
        .def("plan", &HybridAStar::plan, py::arg("start"), py::arg("goal"));

    // ---- MPCTrajectoryPlanner (output refs wrapped → tuple) ----
    py::class_<MPCTrajectoryPlanner>(m, "MPCTrajectoryPlanner")
        .def(py::init<const std::vector<std::vector<int>>&>(),
             py::arg("grid"))
        .def("set_horizon", &MPCTrajectoryPlanner::setHorizon)
        .def("set_dt", &MPCTrajectoryPlanner::setDt)
        .def("set_desired_speed", &MPCTrajectoryPlanner::setDesiredSpeed)
        .def("plan", [](MPCTrajectoryPlanner& self,
                        const std::vector<Pose>& ref_path) {
            std::vector<double> velocities, steers;
            auto traj = self.plan(ref_path, velocities, steers);
            return py::make_tuple(traj, velocities, steers);
        }, py::arg("ref_path"));

    // ---- Map Parser (只暴露 void 返回/Save 函数; 复杂 struct 暂不注册) ----
    m.def("dilate_grid", &dilateGrid, py::arg("grid"), py::arg("radius"));
    m.def("save_grid", &saveGrid,
          py::arg("grid"), py::arg("sr"), py::arg("sc"),
          py::arg("gr"), py::arg("gc"), py::arg("path"));
    m.def("save_grid_ppm", &saveGridPPM,
          py::arg("grid"), py::arg("sr"), py::arg("sc"),
          py::arg("gr"), py::arg("gc"), py::arg("path"));
}
