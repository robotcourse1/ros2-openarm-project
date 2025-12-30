# ROS2 OpenArm 快速开始指南

## 前提条件

- Ubuntu 22.04 (ROS2 Humble) 或 Ubuntu 20.04 (ROS2 Foxy)
- 项目已位于：`~/openarm_ws/src/ros2-openarm-project/`

## 验证工作空间结构

```bash
# 检查项目结构
cd ~/openarm_ws/src/ros2-openarm-project
ls -la

# 应该看到：
# openarm_description/
# motion_control/
# perception/
# calibration/
# system_bringup/
# ...
```

## 快速编译

```bash
# 1. 进入工作空间
cd ~/openarm_ws

# 2. 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译
colcon build

# 4. 配置环境
source install/setup.bash
```

## 快速运行（5个终端）

### 终端1：仿真环境
```bash
cd ~/openarm_ws
source install/setup.bash
ros2 launch openarm_env_bringup mujoco_env_with_bridge.launch.py
```

### 终端2：MoveIt2
```bash
cd ~/openarm_ws
source install/setup.bash
ros2 launch openarm_moveit_config demo.launch.py
```

### 终端3：视觉感知
```bash
cd ~/openarm_ws
source install/setup.bash
ros2 run perception object_detector
```

### 终端4：抓取规划
```bash
cd ~/openarm_ws
source install/setup.bash
# 单臂模式
ros2 launch motion_control grasp.launch.py

# 或双臂模式
# ros2 launch motion_control bimanual_grasp.launch.py
```

### 终端5：触发抓取
```bash
cd ~/openarm_ws
source install/setup.bash
ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.3}" -1
```

## 数据分析

```bash
cd ~/openarm_ws
source install/setup.bash
ros2 run motion_control statistics_analyzer
```

结果保存在：`~/openarm_ws/results/motion_control/`

## 常见问题

### 编译错误：找不到包

确保项目在正确位置：
```bash
ls ~/openarm_ws/src/ros2-openarm-project/openarm_description
```

### 运行时找不到节点

确保已source环境：
```bash
source ~/openarm_ws/install/setup.bash
```

### 数据目录不存在

会自动创建，或手动创建：
```bash
mkdir -p ~/openarm_ws/results/motion_control
```

## 详细文档

- 完整运行指南：`docs/RUN_GUIDE.md`
- 项目结构说明：`docs/PROJECT_STRUCTURE.md`
- 成员C任务：`motion_control/MEMBER_C_COMPLETE.md`
- 路径更新总结：`docs/PATH_UPDATE_SUMMARY.md`

