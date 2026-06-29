# ===== C++ 算法库 (Python 可调用) =====
pybind11_add_module(pnc
    bindings.cpp
    control/mpc/mpc.cc
    control/kf/kf.cc
    control/pid/pid.cc
    control/lqr/lqr.cc
    motion/bicycle_model/bicycle_model.cc
    motion/path/path.cc
    motion/astar/astar.cc
    motion/hybrid_astar/hybrid_astar.cc
    motion/mpc_planner/mpc_planner.cc
    motion/map_parser/map_parser.cc
    motion/safe_corridor/safe_corridor.cc
    motion/bspline/bspline.cc
    prediction/dynamic_obstacle.cc
)

target_link_libraries(pnc PRIVATE Eigen3::Eigen)

# ===== 单元测试 (每个测试独立编译) =====
set(PNC_IMPL_SRCS
    control/mpc/mpc.cc
    control/kf/kf.cc
    control/pid/pid.cc
    control/lqr/lqr.cc
    motion/bicycle_model/bicycle_model.cc
    motion/path/path.cc
    motion/astar/astar.cc
    motion/hybrid_astar/hybrid_astar.cc
    motion/mpc_planner/mpc_planner.cc
    motion/map_parser/map_parser.cc
    motion/safe_corridor/safe_corridor.cc
    motion/bspline/bspline.cc
    prediction/dynamic_obstacle.cc
)

set(PNC_TESTS
    "kf:control/kf/kf_test.cc"
    "pid:control/pid/pid_test.cc"
    "lqr:control/lqr/lqr_test.cc"
    "bicycle_model:motion/bicycle_model/bicycle_model_test.cc"
    "path:motion/path/path_test.cc"
    "astar:motion/astar/astar_test.cc"
    "hybrid_astar:motion/hybrid_astar/hybrid_astar_test.cc"
    "safe_corridor:motion/safe_corridor/safe_corridor_test.cc"
    "bspline:motion/bspline/bspline_test.cc"
)

foreach(entry ${PNC_TESTS})
    string(REPLACE ":" ";" entry_parts "${entry}")
    list(GET entry_parts 0 test_name)
    list(GET entry_parts 1 test_file)
    add_executable(test_${test_name}
        ${PNC_IMPL_SRCS}
        ${test_file}
    )
    target_link_libraries(test_${test_name} PRIVATE Eigen3::Eigen)
endforeach()
