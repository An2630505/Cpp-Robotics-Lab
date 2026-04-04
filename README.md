

1. 编译

make

2. 运行

./output/main

3. python 脚本绘图

python3 plot_results.py

## git提交

0. 拉取远端更新
git fetch
git pull

1. 将文件添加到暂存区

git add .

2. 本地提交

git commit -m "feat[module]： 添加功能"

git commit -m "fix[module]: 修复bug"

git commit -m "refactor[module]: 重构代码"

3. 推送到远端
git push


### 合并分支

1. 切换到对应分支
git checkout <branch>

2. 



## 目录结构

```

 |-build        # 编译产物目录
 |-docs         # 文档
 |-include      # 头文件
 |-output       # 输出结果与数据
 |-src          # 源文件    


## 当前计划

1. 利用增量式PID 控制小球绘制圆轨迹；
    a. 如何设计一个目标轨迹（圆轨迹）；
    b. 如何计算目标轨迹与小球当前位置的误差；
    c. 如何控制小球移动到目标位置；
    d. 打印结果

2. 尝试实现一个卡尔曼滤波算法，并利用滤波算法测量小球观测位置；
    a. 添加小球模型的过程噪声，即控制量输入时，添加一个随机噪声q1；
    b. 添加小球模型的观测噪声，即测量小球位置时，添加一个随机噪声r2；
    c. 实现卡尔曼滤波算法，并用匀速模型去建模小球运动；
    d. 利用匀速卡尔曼滤波模型去观测小球状态；

有空了做，你别心急

我先做哪个