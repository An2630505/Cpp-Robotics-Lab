# 开发操作手册

Cpp-Robotics-Lab 重构后的统一操作流程。所有命令在项目根目录执行。

---

## 环境

```bash
conda activate CRL
```

依赖：Eigen3、pybind11、numpy（已安装，见 refactor-plan.md）。

---

## 构建

```bash
# 首次：配置 + 编译
./build_pnc.sh config

# 日常：只编译（改了 C++ 代码后）
./build_pnc.sh
```

---

## 运行仿真

```bash
# 通用格式
python pipeline/<场景名>.py

# 示例：MPC 车道保持
python pipeline/sim_mpc_basic.py
```

仿真结果输出到 `output/` 目录下同名 `.txt` 文件。

---

## 运行 C++ 单元测试

```bash
# 后续添加，预期格式：
./build_and_test.sh
```

---

## 目录速查

```
pipeline/      — Python 仿真脚本（一个场景一个 .py）
pnc/           — C++ 算法库（.cc + _test.cc）
  motion/      —   运动规划
  control/     —   控制
  prediction/  —   预测（后期）
output/        — 仿真结果
docs/          — 文档
build2/        — CMake 构建产物（已 gitignore）
```

---

## 添加新算法的流程

1. 在 `pnc/<模块>/` 下写 `.cc` 算法文件
2. 在 `pnc/CMakeLists.txt` 中添加源文件
3. 在 `pnc/bindings.cpp` 中添加 pybind11 绑定
4. `./build_pnc.sh` 编译
5. Python 侧 `import pnc` 即可调用

---

> 📅 2026-06-14
