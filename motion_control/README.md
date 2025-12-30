
# Motion Control Package - 成员C的工作成果

本包实现了基于MoveIt2的运动规划与控制功能，用于openarm双臂机器人的抓取任务。

## 功能特性

1. **MoveIt2集成**：使用pymoveit2库实现运动规划接口
2. **OMPL规划器配置**：支持多种OMPL规划算法（RRTConnect、RRT*、PRM等）
3. **抓取状态机**：实现完整的抓取流程（预抓取→抓取→提升→放置→返回）
4. **夹爪控制**：支持夹爪开合控制
5. **物体位置订阅**：订阅`/target_pose`话题获取物体位置


📋 **详细任务完成情况请参考：** [成员C任务完成报告](MEMBER_C_REPORT.md)

## 节点说明

### grasp_planner
主要的抓取规划与控制节点。

**订阅话题：**
- `/target_pose` (geometry_msgs/Point): 物体在base坐标系下的位置

**发布话题：**
- `/grasp_state` (std_msgs/String): 当前抓取状态

**参数：**
- `arm_group` (string, default: "left_arm"): 规划组名称
- `gripper_group` (string, default: "left_hand"): 夹爪组名称
- `base_link` (string, default: "base"): 基座链接名称
- `end_effector_link` (string, default: "openarm_left_link7"): 末端执行器链接名称
- `planner_id` (string, default: "RRTConnectkConfigDefault"): OMPL规划器ID
- `max_velocity` (double, default: 0.3): 最大速度缩放因子
- `max_acceleration` (double, default: 0.3): 最大加速度缩放因子
- `pre_grasp_offset_z` (double, default: 0.15): 预抓取位置高度偏移（米）
- `grasp_offset_z` (double, default: 0.02): 抓取位置高度偏移（米）
- `lift_height` (double, default: 0.2): 提升高度（米）
- `place_position` (double_array, default: [0.5, 0.3, 0.3]): 放置位置 [x, y, z]

### gripper_controller
简单的夹爪控制节点（备用方案）。

## 使用方法

### 1. 编译包

```bash
cd ~/openarm_ws
colcon build --packages-select motion_control
source install/setup.bash
```

### 2. 启动抓取节点

```bash
ros2 launch motion_control grasp.launch.py
```

### 3. 使用自定义参数

```bash
ros2 launch motion_control grasp.launch.py \
    arm_group:=left_arm \
    planner_id:=RRTConnectkConfigDefault \
    max_velocity:=0.5 \
    max_acceleration:=0.5
```

### 4. 测试物体位置发布

```bash
# 发布测试物体位置
ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.3}" -1
```

## 抓取流程

节点实现以下状态机：

1. **IDLE**: 空闲状态，等待物体位置
2. **MOVING_TO_PRE_GRASP**: 移动到预抓取位置（物体上方）
3. **MOVING_TO_GRASP**: 移动到抓取位置（接近物体）
4. **GRASPING**: 闭合夹爪
5. **LIFTING**: 提升物体
6. **MOVING_TO_PLACE**: 移动到放置位置
7. **PLACING**: 打开夹爪放置物体
8. **RETURNING**: 返回初始位置
9. **COMPLETED**: 任务完成

## 依赖

- ROS2 (Humble/Foxy)
- MoveIt2
- pymoveit2 (可选，如果不可用会使用备用方案)
- numpy

## 注意事项

1. 确保MoveIt2已正确配置并运行
2. 确保机器人模型和规划组配置正确
3. 物体位置必须在机器人工作空间内
4. 如果pymoveit2不可用，部分功能可能受限

## 作者

成员C - 运动规划与控制工程师

## 参考资源

- `~/openarm_ws/src/pymoveit2`: MoveIt2 Python接口参考（如果已安装）
- `~/openarm_ws/src/openarm_control`: 控制接口参考（如果已安装）
- `~/openarm_ws/src/openarm_moveit_config`: MoveIt配置参考（如果已安装）

## 数据收集与分析

### 自动数据收集
运行抓取任务时，数据会自动保存到：
```
~/openarm_ws/results/motion_control/
├── trajectories_*.json      # 轨迹数据
└── grasp_results.csv        # 抓取结果
```

### 生成分析图表
```bash
ros2 run motion_control statistics_analyzer
```

生成的输出：
- `smoothness_analysis_*.png` - 轨迹平滑度分析图
- `success_rate_*.png` - 成功率分析图
- `statistics_report_*.txt` - 统计报告

## 参考资源

- `src/pymoveit2`: MoveIt2 Python接口参考
- `src/openarm_control`: 控制接口参考
- `openarm_moveit_config`: MoveIt配置参考

## 相关文档

- [成员C任务完成报告](MEMBER_C_REPORT.md) - 详细的任务完成情况和技术说明


