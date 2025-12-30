# 成员C任务完成报告

**成员：** 林嘉锴  
**角色：** 运动规划与控制工程师  
**完成时间：** 2024年

---

## 任务概述

负责实现基于MoveIt2的运动规划与控制功能，包括OMPL规划器配置、抓取流程实现、双臂协调策略以及数据统计与可视化。

---

## 已完成任务

### 1. MoveIt配置与规划 ✅

#### 1.1 OMPL规划器配置
- **实现位置：** `grasp_planner.py`, `config/planner_config.yaml`
- **功能：**
  - 集成pymoveit2实现MoveIt2接口
  - 支持多种OMPL规划器（RRTConnect、RRT、RRT*、PRM等）
  - 通过参数可配置规划器选择
  - 默认使用RRTConnect规划器（稳定高效）

#### 1.2 约束与时间参数化
- **速度/加速度约束：**
  - 支持通过参数设置 `max_velocity` 和 `max_acceleration`
  - 默认值：0.3（保守值，确保安全执行）
- **时间参数化：**
  - 通过MoveIt2的 `AddTimeOptimalParameterization` 自动处理
  - 确保轨迹满足速度和加速度约束

### 2. 抓取流程实现 ✅

#### 2.1 状态机设计
实现了完整的抓取状态机，包括以下状态：
1. **IDLE** - 空闲状态，等待物体位置
2. **MOVING_TO_PRE_GRASP** - 移动到预抓取位置（物体上方）
3. **MOVING_TO_GRASP** - 移动到抓取位置（接近物体）
4. **GRASPING** - 闭合夹爪进行抓取
5. **LIFTING** - 提升物体到安全高度
6. **MOVING_TO_PLACE** - 移动到放置位置
7. **PLACING** - 打开夹爪放置物体
8. **RETURNING** - 返回初始位置
9. **COMPLETED** - 任务完成

#### 2.2 姿态生成
- **预抓取姿态：** 物体上方，末端执行器垂直向下
- **抓取姿态：** 接近物体，保持垂直向下姿态
- **撤离姿态：** 提升物体，保持姿态稳定

#### 2.3 轨迹执行
- 使用MoveIt2 action执行轨迹
- 支持关节空间规划和笛卡尔路径规划
- 抓取接近阶段使用笛卡尔路径，确保直线下降

#### 2.4 夹爪控制
- 使用 `GripperInterface` 实现夹爪开合
- 支持同步/异步执行
- 提供执行状态等待机制

### 3. 双臂策略 ✅

#### 3.1 双臂配置集成
- 结合 `openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py`
- 支持命名空间配置（`arm_prefix`参数）
- 左右臂控制器分离管理

#### 3.2 智能手臂选择
- 根据物体位置自动选择合适的手臂
- 支持顺序和并行协调模式（可配置）
- 不稳定时优先使用单臂模式

### 4. 数据统计与可视化 ✅

#### 4.1 轨迹分析
- 计算轨迹平滑度（速度/加速度变化率）
- 记录执行时间和路径长度
- 自动保存轨迹数据（JSON格式）

#### 4.2 成功率统计
- 记录每次抓取尝试的结果
- 计算累积成功率
- 按手臂分别统计

#### 4.3 可视化图表
- 轨迹平滑度曲线
- 执行时间曲线
- 成功率统计图

---

## 核心文件

### 主要节点
- **`motion_control/grasp_planner.py`** - 单臂抓取规划与控制节点
  - 实现完整的状态机
  - 集成MoveIt2和夹爪控制
  - 订阅 `/target_pose` 话题，发布 `/grasp_state` 状态

### 配置文件
- **`config/planner_config.yaml`** - OMPL规划器配置参考
- **`launch/grasp.launch.py`** - 单臂抓取启动文件（如存在）

### 辅助文件
- **`motion_control/moveit2_simple.py`** - 简化版MoveIt2接口（备用方案）

---

## 使用方法

### 启动抓取节点

```bash
# 编译包
cd ~/openarm_ws
colcon build --packages-select motion_control
source install/setup.bash

# 启动抓取节点
ros2 launch motion_control grasp.launch.py

# 或使用自定义参数
ros2 launch motion_control grasp.launch.py \
    arm_group:=left_arm \
    planner_id:=RRTConnectkConfigDefault \
    max_velocity:=0.5 \
    max_acceleration:=0.5
```

### 发布物体位置

```bash
# 发布测试物体位置（base坐标系）
ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.3}" -1
```

### 查看抓取状态

```bash
# 监听抓取状态
ros2 topic echo /grasp_state
```

---

## 技术要点

### 规划器选择
- **RRTConnect**：默认选择，双向搜索，速度快且稳定
- **RRT***：最优规划，但计算时间较长
- **PRM**：适合静态环境，需要预处理

### 轨迹平滑
- 通过速度/加速度约束确保轨迹平滑
- 使用笛卡尔路径规划实现直线运动
- 时间参数化自动优化执行时间

### 错误处理
- 规划失败自动进入FAILED状态
- 执行超时检测（默认30秒）
- 夹爪控制失败处理

---

## 接口说明

### 输入接口
- **话题：** `/target_pose` (geometry_msgs/Point)
  - 物体在base坐标系下的位置
  - 来源：perception模块（成员B）

### 输出接口
- **话题：** `/grasp_state` (std_msgs/String)
  - 当前抓取状态
  - 可用于监控和调试

### MoveIt2接口
- **规划组：** `left_arm` / `right_arm`
- **末端执行器：** `openarm_left_link7` / `openarm_right_link7`
- **夹爪组：** `left_hand` / `right_hand`

---

## 论文素材

本实现可用于论文的以下章节：

### 5.5 抓取规划与控制
- **运动规划：** OMPL规划器配置与选择策略
- **状态机设计：** 完整的抓取流程状态机
- **轨迹执行：** MoveIt2 action接口使用

### 实验结果
- **轨迹平滑度：** 速度/加速度变化率分析
- **抓取成功率：** 统计数据和可视化图表
- **执行时间：** 不同规划器的性能对比

---

## 依赖项

- ROS2 (Humble/Foxy)
- MoveIt2
- pymoveit2 (推荐) 或 moveit2_simple (备用)
- numpy

---

## 注意事项

1. 确保MoveIt2已正确配置并运行
2. 确保机器人模型和规划组配置正确
3. 物体位置必须在机器人工作空间内
4. 如果pymoveit2不可用，会使用备用方案

---

## 后续优化建议

1. **双臂协调：** 实现真正的双手协作抓取（如双手同时抓取大物体）
2. **轨迹优化：** 添加轨迹后处理以提高平滑度
3. **错误恢复：** 增强错误处理和自动恢复机制
4. **实时监控：** 添加RViz可视化标记，实时显示规划轨迹

---

**报告完成**

