# ZT_DeliveryUAV：面向边缘的无人机协同配送能量感知调度系统 — 开源发布

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![论文状态](https://img.shields.io/badge/论文-审稿中-blue)]()
[![运行平台](https://img.shields.io/badge/平台-ROS%20Noetic%20%7C%20Ubuntu%2020.04-green)]()
[![仓库地址](https://img.shields.io/badge/GitHub-MrZhou0421%2FZT__DeliveryUAV-black?logo=github)](https://github.com/MrZhou0421/ZT_DeliveryUAV)

本仓库为以下论文的官方开源发布：

> **《面向边缘的无人机协同配送能量资源预估与安全调度架构》**
> 周潼，浙江工业大学，2026年

---

## 项目概述

本仓库包含论文实验结果所用的**核心规划器源代码**与**真实飞行数据集**。系统实现了一套跨层能量感知调度框架，主要包含以下两个层次：

- **微观层（论文第 3.1 & 3.2 节）**：时空解耦轨迹优化器，集成 B 样条转弯能量惩罚项与动态规划（DP）时间重分配模块。
- **宏观层（论文第 3.3 & 3.4 节）**：离线能耗代价张量预编译流水线，以及基于 Google OR-Tools（SCIP 求解器）的边缘侧能量感知 VRP 调度引擎。

---

## 目录结构

```
open_source_release_20260510/
├── README.md               ← 本文件
├── MANIFEST.tsv            ← 完整文件清单（含 SHA-256 校验码，用于复现性审计）
├── LICENSE                 ← 开源许可证
├── flydata/                ← 真实无人机飞行日志（PX4 .ulg 格式）
│   ├── README.md           ← 数据格式与飞行场次说明
│   ├── Hover/              ← 悬停标定飞行（0、1、2 个负载）
│   │   ├── Hover_none_1.ulg
│   │   ├── Hover_none_2.ulg
│   │   ├── Hover_one_1.ulg
│   │   ├── Hover_one_2.ulg
│   │   ├── Hover_two_1.ulg
│   │   └── Hover_two_2.ulg
│   ├── Move_go/            ← 点对点前向飞行数据（23 组不同负载配置）
│   │   ├── none_{1..7}/    ← 无负载（7 次）
│   │   ├── one_{1..4}/     ← 单包（标准 + 46g/70g/138g 变体）
│   │   └── two_{1..4}/     ← 双包（标准 + 多种质量组合）
│   ├── UP_Down/            ← 垂直升降飞行数据
│   └── new/                ← 消融实验对照飞行数据
│       ├── ego/            ← EGO-Planner 基线日志
│       ├── full/           ← 全融合方法（本文提出）日志
│       ├── only_dp/        ← 仅 DP 时间分配（无转弯惩罚）日志
│       └── only_turn/      ← 仅转弯惩罚（无 DP 时间分配）日志
```

> **说明**：规划器 ROS/Catkin 源代码将在论文正式接收后于独立分支发布。`MANIFEST.tsv` 包含所有数据文件的 SHA-256 校验码以保证实验可复现性。

---

## 飞行数据格式

所有飞行日志均为 **PX4 ULog 格式**（`.ulg`），可通过以下工具解析：

- **[pyulog](https://github.com/PX4/pyulog)**（Python 库）：`pip install pyulog`
- **[FlightPlot](https://github.com/PX4/FlightPlot)**（跨平台图形界面）
- **[PX4 在线日志分析](https://logs.px4.io/)**：上传 `.ulg` 文件进行在线可视化

### 文件命名规则

| 文件夹/文件名模式 | 含义 |
|---|---|
| `Hover_none_*.ulg` | 悬停，无负载 |
| `Hover_one_*.ulg` | 悬停，1 个标准负载 |
| `Hover_two_*.ulg` | 悬停，2 个负载 |
| `Move_go/none_*/` | 前向飞行，无负载 |
| `Move_go/one_138g_*/` | 前向飞行，1 个 138g 负载 |
| `Move_go/two_70g_102g_*/` | 前向飞行，2 个负载（70g + 102g）|

负载质量数值对应论文第 2.2.1 节中 Alyassi 代理功率模型所采用的**离散负载分档**。

### 核心 PX4 话题说明

| PX4 话题名 | 用途 |
|---|---|
| `vehicle_local_position` | 三维位置与速度（NED 坐标系） |
| `vehicle_attitude` | 四元数姿态 |
| `actuator_motors` | 各电机 PWM 指令 |
| `battery_status` | 实时电压、电流与累计能耗（真值） |
| `vehicle_acceleration` | IMU 融合三轴加速度 |

---

## 硬件平台

飞行数据在以下**定制四旋翼无人机**平台上采集：

| 组件 | 规格 |
|---|---|
| 飞控 | Pixhawk（PX4 固件） |
| 激光雷达 | Livox Mid-360（固态全向，FAST-LIO2 建图） |
| 机载计算机 | 边缘计算节点（ROS Noetic，Ubuntu 20.04） |
| 最大负载 | 310g（3 个标准包裹） |
| 通信接口 | 通用异步收发器 + 内置 CAN 总线 |

---

## 复现功率模型标定

论文公式 (1) 中的 10 参数 Alyassi 代理模型，基于涵盖多种负载配置的 **35 次真实飞行、21,909 个时序物理采样点**完成标定。复现步骤（脚本将随论文接收后发布）：

```bash
pip install pyulog numpy scikit-learn
# 解析 .ulg 文件并提取特征
python scripts/extract_features.py --data flydata/Move_go/
# 执行 Ridge 回归标定
python scripts/calibrate_model.py
```

---

## 引用

如果本数据集或代码对您的研究有所帮助，请引用：

```bibtex
@article{zhou2026energyaware,
  title   = {面向边缘的无人机协同配送能量资源预估与安全调度架构},
  author  = {周潼},
  journal = {小型微型计算机系统（审稿中）},
  year    = {2026},
  url     = {https://github.com/MrZhou0421/ZT_DeliveryUAV}
}
```

---

## 开源许可证

- **代码**：MIT License（详见 [LICENSE](LICENSE) 文件）
- **飞行数据**（`.ulg` 文件）：[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)（署名-4.0 国际）

---

## 联系方式

**周潼** — 浙江工业大学 计算机科学与技术学院
📧 zhoutong_0421@163.com
