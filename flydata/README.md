# 飞行数据集说明

本目录包含论文《能量最优飞行范式：一种用于无人机配送的跨层方法》所使用的真实无人机飞行日志，均为 PX4 ULog 格式（`.ulg`）。

可使用 [pyulog](https://github.com/PX4/pyulog) 或 [FlightPlot](https://github.com/PX4/FlightPlot) 打开查看。

---

## 各文件夹说明

### `Hover/` — 悬停标定飞行
共 6 组，覆盖 0、1、2 个负载的悬停场景。  
用于提取 Alyassi 代理功率模型标定所需的基础悬停功率（`P_hover`）。

### `Move_go/` — 前向飞行数据
共 23 组，涵盖无负载至双包（多种质量组合）的全部负载配置。每个子文件夹包含一条 `.ulg` 日志。  
子文件夹命名规则：`{包裹数量}_{质量说明}_{运行编号}`

### `UP_Down/` — 垂直升降飞行
用于表征垂直运动（上升/下降）的功率增量特性。

### `new/` — 消融实验对照飞行
对应论文第 5.1 节的四种算法变体：

| 子文件夹 | 对应算法 |
|---|---|
| `ego/` | EGO-Planner（Min-Jerk）基线 |
| `full/` | 全融合方法（本文提出） |
| `only_dp/` | 仅 DP 时间分配（无转弯惩罚） |
| `only_turn/` | 仅转弯惩罚（无 DP 时间分配） |

---

## 核心 PX4 话题

| 话题名 | 用途 |
|---|---|
| `vehicle_local_position` | 位置与速度（用于轨迹复现） |
| `battery_status` | 真值能耗数据 |
| `actuator_motors` | 电机指令（气动分析） |
| `vehicle_acceleration` | IMU 特征（功率模型输入） |

---

## 未包含的数据

以下模型对比标定数据集未纳入本次公开发布包：

- `bemt_energy_model/`
- `gao_energy_model/`
- `shan_energy_model/`

如有科研需要，可通过邮件联系作者获取。
