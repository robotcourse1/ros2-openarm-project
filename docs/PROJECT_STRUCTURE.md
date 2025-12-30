# 项目结构说明

## 标准工作空间结构

项目应该按照以下结构组织：

```
~/openarm_ws/                          # ROS2工作空间根目录
  src/                                 # 源代码目录
    ros2-openarm-project/              # 项目根目录
      ├── openarm_description/         # 机器人模型包（成员A）
      ├── openarm_env_description/     # 环境描述包（成员A）
      ├── openarm_env_bringup/         # 环境启动包（成员A）
      ├── openarm_moveit_config/       # MoveIt配置包
      ├── perception/                  # 视觉感知包（成员B）
      ├── calibration/                 # 手眼标定包（成员B）
      ├── motion_control/              # 运动规划与控制包（成员C）
      ├── system_bringup/              # 系统集成包（成员D）
      ├── docs/                        # 项目文档
      ├── launch/                      # 全局启动文件
      ├── results/                     # 实验结果目录
      ├── setup_workspace.sh           # 工作空间验证脚本
      └── README.md                    # 项目说明
```

## 关键路径说明

### 工作空间路径
- **工作空间根目录**: `~/openarm_ws/`
- **源代码目录**: `~/openarm_ws/src/`
- **项目目录**: `~/openarm_ws/src/ros2-openarm-project/`
- **编译输出**: `~/openarm_ws/install/`
- **构建输出**: `~/openarm_ws/build/`

### 数据存储路径
- **实验结果**: `~/openarm_ws/results/motion_control/`
- **标定结果**: `~/openarm_ws/results/calibration/`（如果存在）

### 环境配置
每次打开新终端需要执行：
```bash
source ~/openarm_ws/install/setup.bash
```

## 验证项目结构

### 方法1：使用验证脚本
```bash
cd ~/openarm_ws/src/ros2-openarm-project
bash setup_workspace.sh ~/openarm_ws
```

### 方法2：手动验证
```bash
# 检查项目目录
cd ~/openarm_ws/src/ros2-openarm-project
ls -la

# 应该看到所有包目录：
# openarm_description/
# motion_control/
# perception/
# calibration/
# system_bringup/
# ...
```

## 如果项目不在正确位置

### 创建符号链接（推荐）
```bash
cd ~/openarm_ws/src
ln -s /path/to/your/actual/project ros2-openarm-project
```

### 直接移动
```bash
mv /path/to/your/actual/project ~/openarm_ws/src/ros2-openarm-project
```

## 编译和运行

### 编译
```bash
cd ~/openarm_ws
colcon build
source install/setup.bash
```

### 运行节点
```bash
# 确保已source环境
source ~/openarm_ws/install/setup.bash

# 运行节点
ros2 launch motion_control grasp.launch.py
```

## 常见路径问题

### 问题1：找不到包
**错误**: `Package 'motion_control' not found`

**解决**:
1. 确认项目在 `~/openarm_ws/src/ros2-openarm-project/`
2. 确认已编译：`cd ~/openarm_ws && colcon build`
3. 确认已source：`source ~/openarm_ws/install/setup.bash`

### 问题2：找不到launch文件
**错误**: `Could not find the 'motion_control' package`

**解决**:
1. 检查 `motion_control/launch/` 目录是否存在
2. 检查 `setup.py` 中是否正确配置了launch文件安装
3. 重新编译：`cd ~/openarm_ws && colcon build --packages-select motion_control`

### 问题3：数据目录不存在
**解决**: 数据目录会自动创建，或手动创建：
```bash
mkdir -p ~/openarm_ws/results/motion_control
```

## 路径检查清单

- [ ] 项目位于 `~/openarm_ws/src/ros2-openarm-project/`
- [ ] 所有包目录存在（openarm_description, motion_control等）
- [ ] 已编译：`cd ~/openarm_ws && colcon build`
- [ ] 已source环境：`source ~/openarm_ws/install/setup.bash`
- [ ] 结果目录可写：`~/openarm_ws/results/`

---

**最后更新**: 2024年

