
该目录主要是构建相关的场景 pipeline

例如 pnc 下就是相关的 C++的算法库，而该目录下就是构建一个场景 pipeline

例如一个车道线保持的控制场景
即 sim_lane_keeping

首先需要去编译 C++相关的算法库

运行
python ./pipeline/sim_lane_keeping.py

运行完会得到


### 1. 编译 C++ 算法库

```bash
# 首次：配置 + 编译
./build_pnc.sh config

# 日常：改了 C++ 代码后只编译
./build_pnc.sh

# 编译 + 运行单元测试
./build_pnc.sh test
```

### 2. 运行仿真场景

```bash
python pipeline/<场景名>.py
```

| 场景 | 脚本 | 说明 |
|------|------|------|
| MPC 基础验证 | `pipeline/sim_mpc_basic.py` | 最小 MPC 车道保持，快速验证 |
| 完整车道保持 | `pipeline/sim_lane_keeping.py` | 复现 `main.cpp`，20段复合路径 |

仿真结果输出到 `output/<场景名>.txt`。

### 3. 可视化

```bash
# 静态图表
python pipeline/<场景名>_visualize.py

# 交互式动画
python pipeline/<场景名>_animate.py

# 保存为图片 / GIF
python pipeline/<场景名>_visualize.py --save output/result.png
python pipeline/<场景名>_animate.py --save output/result.gif
```

## 完整示例 — 车道保持

```bash
# 1. 编译
./build_pnc.sh

# 2. 运行仿真
python pipeline/sim_lane_keeping.py

# 3. 查看结果
python pipeline/sim_lane_keeping_visualize.py
```

## 添加新算法模块

1. 在 `pnc/<模块>/` 下写 `xxx.h` + `xxx.cc`
2. 添加 `xxx_test.cc` 单元测试
3. 在 `pnc/CMakeLists.txt` 中注册源文件和测试
4. 在 `pnc/bindings.cpp` 中添加 pybind11 绑定
5. `./build_pnc.sh test` 编译并跑测试
6. Python 侧 `import pnc; pnc.NewClass()` 即可调用


总的来说这样子就可以实现编译和运行解耦，如果想要调整相关的参数，实际上只需要调整 pipeline 中的 python 脚本而不需要去