#!/bin/bash
# 构建 pnc C++ 算法库
# 用法:
#   ./build_pnc.sh          — 编译
#   ./build_pnc.sh config   — 初次配置 + 编译

set -e
cd "$(dirname "$0")"

EIGEN3_DIR="/opt/homebrew/opt/eigen@3/share/eigen3/cmake"
PYBIND11_DIR="/Users/tory/miniconda3/envs/CRL/lib/python3.11/site-packages/pybind11/share/cmake/pybind11"
PYTHON_EXEC="/Users/tory/miniconda3/envs/CRL/bin/python"
PYTHON_LIB="/Users/tory/miniconda3/envs/CRL/lib/libpython3.11.dylib"
PYTHON_INC="/Users/tory/miniconda3/envs/CRL/include/python3.11"

if [ "$1" = "config" ]; then
    cmake -B build2 \
        -DEigen3_DIR="$EIGEN3_DIR" \
        -Dpybind11_DIR="$PYBIND11_DIR" \
        -DPYTHON_EXECUTABLE="$PYTHON_EXEC" \
        -DPYTHON_LIBRARY="$PYTHON_LIB" \
        -DPYTHON_INCLUDE_DIR="$PYTHON_INC"
fi

cmake --build build2
echo "✅ 构建完成"
