<div align="center">

# Control-Algo-Lab 🚗

**车辆横向控制算法实验室** — 基于自行车模型的 MPC 车道保持仿真平台

</div>

---

## 📋 目录

- [项目概述](#-项目概述)
- [效果图](#-效果图)
- [外部依赖](#-外部依赖)
- [快速开始](#-快速开始)
- [项目结构](#-项目结构)
- [功能特性](#-功能特性)
- [使用指南](#-使用指南)
- [License](#-license)

---

## 🎯 项目概述

**Control-Algo-Lab** 是一个用于学习和验证车辆横向控制算法的仿真平台。采用自行车模型（Bicycle Model）作为被控对象，通过 **模型预测控制（MPC）** 结合 **前馈控制** 实现车道保持（Lane Keeping）功能，支持直线、圆弧、S 弯等多种复杂路径跟踪，并配有卡尔曼滤波进行状态估计。

---

## 🖼️ 效果图

![仿真动画](output.gif)

*运行 `python plot_car_result.py` 可查看动态动画，包含：*

> - 主视图：车辆模型跟踪参考路径的实时动画
> - 右上角小地图：完整路径概览与当前位置
> - 右上子图：横向误差 e_y 随时间变化曲线
> - 右下子图：航向误差 e_psi 随时间变化曲线

---

## 📦 外部依赖

| 依赖 | 版本要求 | 用途 |
|------|---------|------|
| **Eigen3** | ≥ 3.3 | 线性代数库（矩阵运算、QR 分解等） |
| **g++** | ≥ 4.8 (需 C++11 支持) | C++ 编译器 |
| **Python** | ≥ 3.6 | 数据可视化 |
| **matplotlib** | ≥ 3.0 | Python 绘图库 |
| **numpy** | ≥ 1.18 | Python 数值计算库 |

### macOS 安装依赖

```bash
# 安装 Eigen3
brew install eigen

# 安装 Python 依赖
pip3 install matplotlib numpy
```

### Ubuntu 安装依赖

```bash
# 安装 Eigen3
sudo apt install libeigen3-dev

# 安装 Python 依赖
pip3 install matplotlib numpy
```

---

## 🚀 快速开始

### 1️⃣ 编译

```bash
make
```

> 默认使用 `g++`，C++11 标准。如需自定义编译器，修改 `Makefile` 中的 `CXX` 变量。

### 2️⃣ 运行仿真

```bash
./build/main
```

仿真结果将输出到 `output/output.txt`，包含每步的时间步、误差状态和控制量。

### 3️⃣ 可视化

```bash
# 动态动画（推荐）
python plot_car_result.py

# 静态结果图
python plot_results.py
```

### 一键运行

```bash
./run.sh
```

等价于依次执行 `make` → `./build/main` → `python plot_results.py`。

---

## 📁 项目结构

```
.
├── Makefile            # 编译配置
├── config.json         # 仿真参数配置
├── main.cpp            # 主程序入口
├── main.h              # 主程序头文件
├── run.sh              # 一键运行脚本
│
├── include/            # 头文件
│   ├── Path.h          #   多段组合路径（直道/圆弧/S弯）
│   ├── Plant_car.h     #   自行车模型被控对象
│   ├── MPC.h           #   模型预测控制器
│   ├── KF.h            #   卡尔曼滤波器
│   ├── PID.h           #   PID 控制器
│   ├── LQR.h           #   LQR 控制器
│   ├── Object.h        #   参考目标对象
│   └── Plant.h         #   通用被控对象（旧版）
│
├── src/                # 源文件
│   ├── Path.cpp
│   ├── Plant_car.cpp
│   ├── MPC.cpp
│   ├── KF.cpp
│   ├── PID.cpp
│   ├── LQR.cpp
│   ├── Object.cpp
│   └── Plant.cpp
│
├── build/              # 编译产物
├── output/             # 仿真输出与图表
│   ├── output.txt      #   仿真数据
│   └── simulation_results.png
│
└── plot_car_result.py  # 动态可视化脚本
    plot_results.py
    plot_results_animation.py
```

---

## ✨ 功能特性

### 车辆模型
- **自行车模型**（Bicycle Model）：两轮车辆动力学模型，包含侧偏刚度、轴距等物理参数
- **过程/观测噪声**：仿真真实传感器噪声环境
- **卡尔曼滤波**：对含噪声的状态进行最优估计

### 路径定义
- **直道**（Straight）：沿任意航向的直线
- **圆弧**（Arc）：任意半径/角度的左转或右转，支持完整圆形（360°）
- **S 弯**（Slalom）：正弦型曲线，可配置幅值和频率
- **多段组合**：以上三种路段任意拼接，自动计算衔接位姿

### 控制器
- **MPC**（模型预测控制）：基于线性误差模型的预测控制，带约束处理能力
- **前馈控制**：根据路径曲率计算前馈补偿，减少稳态误差
- **PID** / **LQR**：可选的传统控制方法（代码保留）

### 可视化
- **主视图**：车辆模型实时跟踪状态（车身、前后轮、车道边界）
- **小地图**：完整路径俯瞰 + 当前位置标记
- **误差曲线**：横向误差 e_y 和航向误差 e_psi 实时绘制

---

## 📖 使用指南

### 自定义路径

编辑 `main.cpp` 中的路径定义：

```cpp
Path path;
path.addStraight(80.0f);          // 直道 80m
path.addArc(75.4f, 12.0f);        // 圆弧（半径12m，360°完整圆形）
path.addSlalom(120.0f, 8.0f, 0.1f); // S 弯（幅值8m，角频率0.1 rad/m）
path.build();                     // 构建路径（计算各段起始位姿）
```

支持的路段类型：
| 方法 | 参数 | 说明 |
|------|------|------|
| `addStraight(length)` | 长度(m) | 沿当前航向的直线 |
| `addArc(length, radius)` | 弧长(m), 半径(m) | radius>0 左转，<0 右转 |
| `addSlalom(length, A, omega)` | 长度(m), 幅值(m), 角频率(rad/m) | 正弦型 S 弯 |

### 调整控制器参数

编辑 `main.cpp` 中 `MPCInit` 函数的权重矩阵：

```cpp
Q(0,0) = 100;   // e_y 权重（横向误差）
Q(1,1) = 1;     // de_y 权重
Q(2,2) = 20;    // e_psi 权重（航向误差）
Q(3,3) = 1;     // de_psi 权重
R(0,0) = 0.05;  // 控制量权重
```

### 配置仿真参数

编辑 `config.json`：

```json
{
  "simulation": {
    "dt": 0.1,
    "total_steps": 800
  }
}
```

---

## 📄 License

MIT
