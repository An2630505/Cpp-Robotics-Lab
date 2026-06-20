#!/bin/bash
# 构建 + 测试 pnc C++ 算法库
# 用法:
#   ./build_pnc.sh           — 编译
#   ./build_pnc.sh config    — 初次配置 + 编译
#   ./build_pnc.sh test      — 编译 + 运行单元测试
set -e
cd "$(dirname "$0")"

EIGEN3_DIR="/opt/homebrew/opt/eigen@3/share/eigen3/cmake"
PYBIND11_DIR="/Users/tory/miniconda3/envs/CRL/lib/python3.11/site-packages/pybind11/share/cmake/pybind11"
PYTHON_EXEC="/Users/tory/miniconda3/envs/CRL/bin/python"
PYTHON_LIB="/Users/tory/miniconda3/envs/CRL/lib/libpython3.11.dylib"
PYTHON_INC="/Users/tory/miniconda3/envs/CRL/include/python3.11"

if [ "$1" = "config" ]; then
    cmake -B build \
        -DEigen3_DIR="$EIGEN3_DIR" \
        -Dpybind11_DIR="$PYBIND11_DIR" \
        -DPYTHON_EXECUTABLE="$PYTHON_EXEC" \
        -DPYTHON_LIBRARY="$PYTHON_LIB" \
        -DPYTHON_INCLUDE_DIR="$PYTHON_INC"
elif [ "$1" = "test" ]; then
    cmake --build build
    echo ""
    for t in kf pid lqr bicycle_model path astar hybrid_astar safe_corridor bspline; do
        echo "=== $t ==="
        ./build/pnc/test_$t
    done
    echo ""
    echo "✅ 全部测试通过"
    exit 0
fi

cmake --build build
echo "✅ 构建完成"
