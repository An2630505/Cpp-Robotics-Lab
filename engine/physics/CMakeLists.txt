# ===== Engine Physics Layer (pybind11 module) =====

pybind11_add_module(engine_physics
    bindings.cpp
    motion_model.cc
    collision.cc
    physics_world.cc
)

# engine/physics is independent of pnc — no Eigen3 or other project deps
