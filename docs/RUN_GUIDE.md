# ROS2 OpenArm 项目完整运行指南

## 目录
1. [环境准备](#环境准备)
2. [工作空间设置](#工作空间设置)
3. [编译项目](#编译项目)
4. [运行系统](#运行系统)
5. [数据收集与分析](#数据收集与分析)
6. [常见问题](#常见问题)

---

## 环境准备

### 1. 系统要求
- Ubuntu 22.04 (ROS2 Humble) 或 Ubuntu 20.04 (ROS2 Foxy)
- Python 3.8+
- MuJoCo 2.x 或 3.x

### 2. 安装ROS2依赖

```bash
# 更新包列表
sudo apt update

# 安装ROS2基础包（如果未安装）
sudo apt install ros-${ROS_DISTRO}-desktop

# 安装MoveIt2
sudo apt install ros-${ROS_DISTRO}-moveit

# 安装其他依赖
sudo apt install \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    python3-pip

# 安装Python依赖
pip3 install numpy matplotlib transforms3d scipy
```

### 3. 安装pymoveit2（可选但推荐）

```bash
cd ~/openarm_ws/src
# 如果src目录下已有pymoveit2，跳过此步
# 否则从GitHub克隆
git clone https://github.com/JafarAbdi/pymoveit2.git
```

---

## 工作空间设置

### 正确的目录结构

项目应该位于以下结构：
```
~/openarm_ws/
  src/
    ros2-openarm-project/          # 项目根目录
      openarm_description/         # 机器人模型
      motion_control/              # 运动规划与控制
      perception/                  # 视觉感知
      calibration/                 # 手眼标定
      system_bringup/              # 系统集成
      openarm_env_bringup/         # 环境启动
      openarm_env_description/     # 环境描述
      ...
```

### 验证工作空间结构

```bash
# 验证项目目录结构
cd ~/openarm_ws/src/ros2-openarm-project
ls -la

# 应该看到所有包目录
# 如果项目已经在正确位置，直接进入下一步编译
```

### 使用验证脚本

```bash
# 如果项目中有setup_workspace.sh，可以运行验证
cd ~/openarm_ws/src/ros2-openarm-project
bash setup_workspace.sh ~/openarm_ws
```

---

## 编译项目

### 1. 安装依赖

```bash
cd ~/openarm_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 编译

```bash
cd ~/openarm_ws
colcon build --symlink-install
```

### 3. 配置环境

```bash
source install/setup.bash
```

**注意**：每次打开新终端都需要执行 `source install/setup.bash`

---

## 运行系统

### 完整系统启动流程

#### 步骤1：启动仿真环境（成员A的工作）

```bash
# 终端1：启动MuJoCo仿真环境
ros2 launch openarm_env_bringup mujoco_env_with_bridge.launch.py
```

#### 步骤2：启动MoveIt2（成员C需要）

```bash
# 终端2：启动MoveIt2（单臂）
ros2 launch openarm_moveit_config demo.launch.py

# 或双臂配置
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

#### 步骤3：启动视觉感知（成员B的工作）

```bash
# 终端3：启动视觉检测节点
ros2 launch perception perception.launch.py

# 或直接运行
ros2 run perception object_detector
```

#### 步骤4：启动运动规划与控制（成员C的工作）

**单臂模式：**

```bash
# 终端4：启动单臂抓取规划
ros2 launch motion_control grasp.launch.py
```

**双臂模式：**

```bash
# 终端4：启动双臂抓取规划
ros2 launch motion_control bimanual_grasp.launch.py \
    use_bimanual:=true \
    coordination_mode:=sequential
```

#### 步骤5：触发抓取（测试）

```bash
# 终端5：发布测试物体位置
ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.3}" -1

# 或使用测试脚本
ros2 run motion_control test_grasp
```

---

## 数据收集与分析

### 1. 运行抓取任务收集数据

数据会自动保存到 `~/openarm_ws/results/motion_control/` 目录：
- `trajectories_*.json`: 轨迹数据
- `grasp_results.csv`: 抓取结果

### 2. 生成分析图表和报告

```bash
# 生成所有分析图表和报告
ros2 run motion_control statistics_analyzer

# 或直接运行Python脚本
python3 ~/openarm_ws/src/ros2-openarm-project/motion_control/motion_control/statistics_analyzer.py
```

生成的输出文件：
- `smoothness_analysis_*.png`: 轨迹平滑度分析图
- `success_rate_*.png`: 成功率分析图
- `statistics_report_*.txt`: 统计报告

### 3. 查看实时状态

```bash
# 查看抓取状态
ros2 topic echo /grasp_state

# 查看双臂抓取状态
ros2 topic echo /bimanual_grasp_state

# 查看关节状态
ros2 topic echo /joint_states

# 查看物体位置
ros2 topic echo /target_pose
```

---

## 常见问题

### 问题1：编译错误 - 找不到openarm_description

**错误信息：**
```
CMake Error: The source directory "/home/kevin/openarm_ws/src/ros2-openarm-project/openarm_description" does not exist.
```

**解决方法：**
```bash
# 检查项目目录结构
cd ~/openarm_ws/src/ros2-openarm-project
ls -la

# 确认openarm_description目录存在
# 如果不存在，检查项目是否完整
```

### 问题2：MoveIt2无法连接

**检查：**
```bash
# 检查MoveIt2服务
ros2 service list | grep moveit

# 检查规划组
ros2 param get /move_group planning_group
```

### 问题3：pymoveit2导入失败

**解决方法：**
```bash
# 确保pymoveit2已编译
cd ~/openarm_ws
colcon build --packages-select pymoveit2
source install/setup.bash

# 或使用备用方案（代码会自动降级）
```

### 问题4：轨迹执行失败

**可能原因：**
1. 目标位置超出工作空间
2. 规划器参数不合适
3. 碰撞检测失败

**调试：**
```bash
# 降低速度
ros2 launch motion_control grasp.launch.py max_velocity:=0.2

# 更换规划器
ros2 launch motion_control grasp.launch.py planner_id:=RRTstarkConfigDefault
```

### 问题5：夹爪不响应

**检查：**
```bash
# 检查夹爪控制器
ros2 action list | grep gripper

# 手动测试夹爪
ros2 run motion_control gripper_controller
```

---

## 项目结构说明

实际工作空间结构：
```
~/openarm_ws/
  src/
    ros2-openarm-project/          # 项目根目录
      ├── openarm_description/      # 机器人模型（成员A）- CMake包
      ├── openarm_moveit_config/    # MoveIt配置（成员C）- CMake包 ⭐
      ├── openarm_env_description/  # 环境模型（成员A）- CMake包
      ├── openarm_env_bringup/      # 环境启动（成员A）- Python包
      ├── perception/               # 视觉感知（成员B）- Python包
      ├── calibration/              # 手眼标定（成员B）- Python包
      ├── motion_control/           # 运动规划与控制（成员C）⭐ - Python包
      │   ├── motion_control/
      │   │   ├── grasp_planner.py           # 单臂抓取
      │   │   ├── bimanual_grasp_planner.py  # 双臂抓取
      │   │   ├── statistics_analyzer.py    # 数据分析
      │   │   └── ...
      │   └── launch/
      │       ├── grasp.launch.py
      │       └── bimanual_grasp.launch.py
      └── system_bringup/           # 系统集成（成员D）- Python包
```

---

## 快速开始示例

```bash
# 1. 验证工作空间结构
cd ~/openarm_ws/src/ros2-openarm-project
ls -la  # 确认所有包都在

# 2. 编译
cd ~/openarm_ws
colcon build
source install/setup.bash

# 3. 启动系统（需要多个终端）
# 终端1: 仿真环境
ros2 launch openarm_env_bringup mujoco_env_with_bridge.launch.py

# 终端2: MoveIt2
ros2 launch openarm_moveit_config demo.launch.py

# 终端3: 视觉感知
ros2 run perception object_detector

# 终端4: 抓取规划
ros2 launch motion_control grasp.launch.py

# 终端5: 触发抓取
ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.3}" -1

# 4. 分析数据
ros2 run motion_control statistics_analyzer
```

---

## 成员C任务检查清单

- [x] MoveIt配置与规划（OMPL规划器）
- [x] 抓取流程（预抓取/抓取/撤离）
- [x] 夹爪开合控制
- [x] 单臂实现
- [x] 双臂协调实现
- [x] 轨迹平滑度分析
- [x] 成功率数据统计
- [x] 曲线图生成
- [x] 统计报告生成

---

## 联系与支持

如有问题，请检查：
1. 各模块的README文档
2. ROS2官方文档
3. MoveIt2官方文档

---

**最后更新**: 2024年
