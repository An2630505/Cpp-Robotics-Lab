#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "types.h"
#include "motion_model.h"
#include "collision.h"
#include "physics_world.h"

namespace py = pybind11;
using namespace engine::physics;

// MotionModel 的 trampoline 类 (支持 Python 子类化)
class PyMotionModel : public MotionModel {
public:
    Velocity step(const Pose& current_pose,
                  const Velocity& current_vel,
                  const ControlInput& cmd,
                  double dt) override {
        PYBIND11_OVERRIDE_PURE(Velocity, MotionModel, step,
                               current_pose, current_vel, cmd, dt);
    }
};

PYBIND11_MODULE(engine_physics, m) {
    m.doc() = "Engine Physics Layer — 2D simulation physics core";

    // ---- Vec2d ----
    py::class_<Vec2d>(m, "Vec2d")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &Vec2d::x)
        .def_readwrite("y", &Vec2d::y)
        .def("__add__", [](const Vec2d& a, const Vec2d& b) { return a + b; })
        .def("__sub__", [](const Vec2d& a, const Vec2d& b) { return a - b; })
        .def("__mul__", [](const Vec2d& v, double s) { return v * s; })
        .def("__truediv__", [](const Vec2d& v, double s) { return v / s; })
        .def("dot", &Vec2d::dot)
        .def("length", &Vec2d::length)
        .def("normalized", &Vec2d::normalized)
        .def("__repr__", [](const Vec2d& v) {
            return "Vec2d(x=" + std::to_string(v.x) + ", y=" + std::to_string(v.y) + ")";
        });

    // ---- Pose ----
    py::class_<Pose>(m, "Pose")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("x"), py::arg("y"), py::arg("theta"))
        .def_readwrite("x", &Pose::x)
        .def_readwrite("y", &Pose::y)
        .def_readwrite("theta", &Pose::theta)
        .def("transform", &Pose::transform, py::arg("local"))
        .def("__repr__", [](const Pose& p) {
            return "Pose(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) +
                   ", theta=" + std::to_string(p.theta) + ")";
        });

    // ---- Velocity ----
    py::class_<Velocity>(m, "Velocity")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("vx"), py::arg("vy"), py::arg("omega"))
        .def_readwrite("vx", &Velocity::vx)
        .def_readwrite("vy", &Velocity::vy)
        .def_readwrite("omega", &Velocity::omega)
        .def("__repr__", [](const Velocity& v) {
            return "Velocity(vx=" + std::to_string(v.vx) + ", vy=" +
                   std::to_string(v.vy) + ", omega=" + std::to_string(v.omega) + ")";
        });

    // ---- Polygon ----
    py::class_<Polygon>(m, "Polygon")
        .def(py::init<>())
        .def(py::init<std::vector<Vec2d>>(), py::arg("vertices"))
        .def_readwrite("vertices", &Polygon::vertices)
        .def("area", &Polygon::area)
        .def_static("aabb", &Polygon::aabb,
                    py::arg("half_w"), py::arg("half_h"))
        .def_static("vehicle", &Polygon::vehicle,
                    py::arg("half_width"), py::arg("forward"), py::arg("backward"))
        .def("__repr__", [](const Polygon& p) {
            return "Polygon(vertices=" + std::to_string(p.vertices.size()) + ")";
        });

    // ---- ControlInput ----
    py::class_<ControlInput>(m, "ControlInput")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("steer"), py::arg("ax"))
        .def_readwrite("steer", &ControlInput::steer)
        .def_readwrite("ax", &ControlInput::ax)
        .def("__repr__", [](const ControlInput& c) {
            return "ControlInput(steer=" + std::to_string(c.steer) +
                   ", ax=" + std::to_string(c.ax) + ")";
        });

    // ---- EntityState ----
    py::class_<EntityState>(m, "EntityState")
        .def(py::init<>())
        .def_readwrite("id", &EntityState::id)
        .def_readwrite("pose", &EntityState::pose)
        .def_readwrite("vel", &EntityState::vel)
        .def_readwrite("geometry", &EntityState::geometry)
        .def_readwrite("mass", &EntityState::mass)
        .def_readwrite("is_static", &EntityState::is_static)
        .def("__repr__", [](const EntityState& s) {
            return "EntityState(id=" + std::to_string(s.id) +
                   ", pose=" + std::to_string(s.pose.x) + "," +
                   std::to_string(s.pose.y) + ")";
        });

    // ---- CollisionResult ----
    py::class_<CollisionResult>(m, "CollisionResult")
        .def(py::init<>())
        .def_readwrite("collides", &CollisionResult::collides)
        .def_readwrite("normal", &CollisionResult::normal)
        .def_readwrite("penetration", &CollisionResult::penetration)
        .def_readwrite("contact_point", &CollisionResult::contact_point)
        .def("__repr__", [](const CollisionResult& r) {
            return r.collides
                ? "CollisionResult(collides=True, pen=" + std::to_string(r.penetration) + ")"
                : "CollisionResult(collides=False)";
        });

    // ---- CollisionEvent ----
    py::class_<CollisionEvent>(m, "CollisionEvent")
        .def(py::init<>())
        .def_readwrite("entity_a", &CollisionEvent::entity_a)
        .def_readwrite("entity_b", &CollisionEvent::entity_b)
        .def_readwrite("result", &CollisionEvent::result)
        .def("__repr__", [](const CollisionEvent& e) {
            return "CollisionEvent(a=" + std::to_string(e.entity_a) +
                   ", b=" + std::to_string(e.entity_b) +
                   ", " + std::to_string(e.result.collides) + ")";
        });

    // ---- MotionModel (base, with trampoline) ----
    py::class_<MotionModel, PyMotionModel, std::shared_ptr<MotionModel>>(m, "MotionModel")
        .def(py::init<>())
        .def("step", &MotionModel::step,
             py::arg("current_pose"), py::arg("current_vel"),
             py::arg("cmd"), py::arg("dt"));

    // ---- BicycleModel ----
    py::class_<BicycleModel, MotionModel, std::shared_ptr<BicycleModel>>(m, "BicycleModel")
        .def(py::init<double>(), py::arg("wheelbase"))
        .def_property_readonly("wheelbase", &BicycleModel::wheelbase)
        .def_property("lat_damping", &BicycleModel::lat_damping,
                      &BicycleModel::set_lat_damping,
                      "Lateral damping coefficient (1/s, default 5.0, 0=ice)");

    // ---- SimpleModel ----
    py::class_<SimpleModel, MotionModel, std::shared_ptr<SimpleModel>>(m, "SimpleModel")
        .def(py::init<>());

    // ---- PhysicsWorld ----
    py::class_<PhysicsWorld>(m, "PhysicsWorld")
        .def(py::init<>())
        .def("add_entity", &PhysicsWorld::add_entity,
             py::arg("state"), py::arg("model"),
             "Add an entity and return its ID")
        .def("remove_entity", &PhysicsWorld::remove_entity,
             py::arg("id"))
        .def("get_entity_state", &PhysicsWorld::get_entity_state,
             py::arg("id"), py::return_value_policy::reference_internal)
        .def("get_all_entity_ids", &PhysicsWorld::get_all_entity_ids)
        .def("get_all_states", &PhysicsWorld::get_all_states)
        .def("entity_count", &PhysicsWorld::entity_count)
        .def("apply_control", &PhysicsWorld::apply_control,
             py::arg("id"), py::arg("cmd"))
        .def("step", &PhysicsWorld::step, py::arg("dt"),
             "Advance simulation by dt seconds")
        .def("get_collisions", &PhysicsWorld::get_collisions,
             "Get collision events from the last step");

    // ---- 自由函数 ----
    m.def("sat_collision", &sat_collision,
          py::arg("poly_a"), py::arg("pose_a"),
          py::arg("poly_b"), py::arg("pose_b"),
          "SAT collision detection between two convex polygons");
}
