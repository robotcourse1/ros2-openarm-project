# 成员C任务验证指南

本指南帮助您验证运动规划与控制功能是否正常工作。

## 📋 验证前准备

### 1. 确保包已编译

```bash
cd ~/openarm_ws
colcon build --packages-select motion_control
source install/setup.bash
```

### 2. 检查包是否正确安装

```bash
# 检查包是否存在
ros2 pkg list | grep motion_control

# 检查可执行文件
ros2 run motion_control --help 2>&1 | head -20

# 应该看到：
# grasp_planner
# bimanual_grasp_planner
# gripper_controller
# test_grasp
# statistics_analyzer
```

### 3. 检查launch文件

```bash
# 检查launch文件是否存在
ros2 pkg prefix motion_control
# 输出路径后，检查：
ls $(ros2 pkg prefix motion_control)/share/motion_control/launch/

# 应该看到：
# grasp.launch.py
# bimanual_grasp.launch.py
```

---

## ✅ 验证步骤

### 步骤1：验证代码结构

```bash
cd ~/openarm_ws/src/ros2-openarm-project/motion_control

# 检查关键文件是否存在
ls -la motion_control/
# 应该看到：
# - grasp_planner.py
# - bimanual_grasp_planner.py
# - gripper_controller.py
# - statistics_analyzer.py
# - test_grasp.py

# 检查launch文件
ls -la launch/
# 应该看到：
# - grasp.launch.py
# - bimanual_grasp.launch.py
```

**✅ 通过标准**：所有文件都存在

---

### 步骤2：验证代码语法（静态检查）

```bash
cd ~/openarm_ws/src/ros2-openarm-project/motion_control

# 检查Python语法
python3 -m py_compile motion_control/grasp_planner.py
python3 -m py_compile motion_control/bimanual_grasp_planner.py
python3 -m py_compile motion_control/statistics_analyzer.py

# 如果没有错误输出，说明语法正确
echo "语法检查完成"
```

**✅ 通过标准**：无语法错误

---

### 步骤3：验证节点可以启动（不需要MoveIt2）

```bash
# 终端1：启动节点（会等待MoveIt2，但可以验证节点本身是否正常）
ros2 run motion_control grasp_planner --ros-args --log-level debug

# 应该看到：
# [INFO] [grasp_planner]: 抓取规划节点已启动 (规划组: left_arm, 规划器: RRTConnectkConfigDefault)
# [WARN] [grasp_planner]: MoveIt2 not available! 或等待MoveIt2连接

# 按Ctrl+C停止
```

**✅ 通过标准**：节点可以启动，没有Python导入错误

---

### 步骤4：验证数据分析工具（独立运行）

```bash
# 创建测试数据目录
mkdir -p ~/openarm_ws/results/motion_control

# 运行数据分析工具（即使没有数据也会运行）
ros2 run motion_control statistics_analyzer

# 应该看到：
# 开始生成统计分析...
# 1. 生成轨迹平滑度分析图...
# 2. 生成成功率分析图...
# 3. 生成统计报告...
# 统计摘要
# ============================================================
# 成功率: 0.00%
# 总轨迹数: 0
# ...
```

**✅ 通过标准**：工具可以运行，生成空报告（因为没有数据）

---

### 步骤5：验证Launch文件语法

```bash
cd ~/openarm_ws
source install/setup.bash

# 检查launch文件语法（不实际启动）
ros2 launch motion_control grasp.launch.py --show-args

# 应该显示所有可配置参数，没有错误

# 检查双臂launch文件
ros2 launch motion_control bimanual_grasp.launch.py --show-args
```

**✅ 通过标准**：launch文件语法正确，可以显示参数

---

### 步骤6：验证功能完整性（代码检查）

```bash
cd ~/openarm_ws/src/ros2-openarm-project/motion_control

# 检查关键功能是否存在
echo "=== 检查单臂抓取功能 ==="
grep -n "class GraspPlanner" motion_control/grasp_planner.py
grep -n "move_to_pose\|move_to_configuration" motion_control/grasp_planner.py
grep -n "MOVING_TO_PRE_GRASP\|GRASPING\|LIFTING" motion_control/grasp_planner.py

echo "=== 检查双臂功能 ==="
grep -n "class BimanualGraspPlanner" motion_control/bimanual_grasp_planner.py
grep -n "left_moveit2\|right_moveit2" motion_control/bimanual_grasp_planner.py

echo "=== 检查数据分析功能 ==="
grep -n "class StatisticsAnalyzer" motion_control/statistics_analyzer.py
grep -n "plot_trajectory_smoothness\|plot_success_rate" motion_control/statistics_analyzer.py
```

**✅ 通过标准**：所有关键类和函数都存在

---

### 步骤7：验证参数配置

```bash
cd ~/openarm_ws
source install/setup.bash

# 测试使用自定义参数启动
ros2 launch motion_control grasp.launch.py \
    arm_group:=left_arm \
    planner_id:=RRTConnectkConfigDefault \
    max_velocity:=0.5 \
    max_acceleration:=0.5 \
    --show-args

# 应该显示所有参数，包括自定义值
```

**✅ 通过标准**：参数可以正确配置

---

### 步骤8：验证数据保存路径

```bash
# 检查代码中的数据路径
cd ~/openarm_ws/src/ros2-openarm-project/motion_control

# 检查结果目录路径
grep -n "results_dir\|openarm_ws.*results" motion_control/*.py

# 应该看到路径设置为：~/openarm_ws/results/motion_control
```

**✅ 通过标准**：数据路径配置正确

---

## 🧪 完整功能测试（需要MoveIt2运行）

### 前提条件
- MoveIt2服务已运行
- 机器人模型已加载
- 关节状态发布正常

### 测试1：单臂抓取节点

```bash
# 终端1：启动MoveIt2（如果还没有）
# ros2 launch openarm_moveit_config demo.launch.py

# 终端2：启动抓取规划节点
cd ~/openarm_ws
source install/setup.bash
ros2 launch motion_control grasp.launch.py

# 应该看到：
# [INFO] [grasp_planner]: 抓取规划节点已启动
# [INFO] [grasp_planner]: 等待MoveIt2连接...

# 终端3：发布测试物体位置
ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.3}" -1

# 观察终端2的输出，应该看到状态变化
```

**✅ 通过标准**：
- 节点成功启动
- 可以接收物体位置
- 状态机开始工作

### 测试2：检查状态发布

```bash
# 在抓取节点运行时，检查状态话题
ros2 topic echo /grasp_state

# 应该看到状态消息：
# data: idle
# data: moving_to_pre_grasp
# data: moving_to_grasp
# ...
```

**✅ 通过标准**：状态话题正常发布

### 测试3：验证数据记录

```bash
# 运行几次抓取任务后，检查数据是否保存
ls -la ~/openarm_ws/results/motion_control/

# 应该看到：
# - trajectories_*.json（如果有轨迹数据）
# - grasp_results.csv（如果有抓取结果）
```

**✅ 通过标准**：数据文件被创建

### 测试4：生成分析图表

```bash
# 确保有一些测试数据后
ros2 run motion_control statistics_analyzer

# 检查生成的图表
ls -la ~/openarm_ws/results/motion_control/*.png
ls -la ~/openarm_ws/results/motion_control/*.txt

# 应该看到：
# - smoothness_analysis_*.png
# - success_rate_*.png
# - statistics_report_*.txt
```

**✅ 通过标准**：图表和报告成功生成

---

## 📊 功能验证清单

### 核心功能
- [ ] 代码文件存在且语法正确
- [ ] 节点可以启动（不报Python错误）
- [ ] Launch文件语法正确
- [ ] 参数可以配置
- [ ] 数据路径配置正确

### 单臂抓取
- [ ] GraspPlanner类存在
- [ ] MoveIt2接口集成
- [ ] 状态机完整（9个状态）
- [ ] 夹爪控制逻辑存在
- [ ] 可以接收/target_pose话题

### 双臂抓取
- [ ] BimanualGraspPlanner类存在
- [ ] 双臂MoveIt2接口初始化
- [ ] 协调模式可配置
- [ ] 状态机支持双臂

### 数据分析
- [ ] StatisticsAnalyzer类存在
- [ ] 轨迹平滑度计算
- [ ] 成功率统计
- [ ] 图表生成功能
- [ ] 报告生成功能

### 交付物
- [ ] 所有代码文件存在
- [ ] Launch文件存在
- [ ] 配置文件存在
- [ ] 文档完整

---

## 🚨 常见问题排查

### 问题1：节点无法启动

```bash
# 检查Python依赖
python3 -c "import rclpy; import numpy; import matplotlib; print('OK')"

# 检查pymoveit2（可选）
python3 -c "from pymoveit2 import MoveIt2; print('pymoveit2 OK')" 2>&1
# 如果没有安装，会显示警告，但不影响基本功能
```

### 问题2：找不到launch文件

```bash
# 重新编译
cd ~/openarm_ws
colcon build --packages-select motion_control
source install/setup.bash

# 检查安装
ros2 pkg prefix motion_control
ls $(ros2 pkg prefix motion_control)/share/motion_control/launch/
```

### 问题3：数据目录无法创建

```bash
# 手动创建并检查权限
mkdir -p ~/openarm_ws/results/motion_control
chmod 755 ~/openarm_ws/results/motion_control
ls -ld ~/openarm_ws/results/motion_control
```

---

## ✅ 快速验证脚本

创建一个快速验证脚本：

```bash
#!/bin/bash
# 快速验证脚本

echo "=== 成员C任务验证 ==="
echo ""

echo "1. 检查文件..."
cd ~/openarm_ws/src/ros2-openarm-project/motion_control
files=("motion_control/grasp_planner.py" 
       "motion_control/bimanual_grasp_planner.py"
       "motion_control/statistics_analyzer.py"
       "launch/grasp.launch.py"
       "launch/bimanual_grasp.launch.py")

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (缺失)"
    fi
done

echo ""
echo "2. 检查Python语法..."
python3 -m py_compile motion_control/grasp_planner.py 2>&1 && echo "  ✓ grasp_planner.py"
python3 -m py_compile motion_control/bimanual_grasp_planner.py 2>&1 && echo "  ✓ bimanual_grasp_planner.py"
python3 -m py_compile motion_control/statistics_analyzer.py 2>&1 && echo "  ✓ statistics_analyzer.py"

echo ""
echo "3. 检查包安装..."
cd ~/openarm_ws
source install/setup.bash 2>/dev/null
if ros2 pkg list | grep -q motion_control; then
    echo "  ✓ motion_control包已安装"
else
    echo "  ✗ motion_control包未安装（需要编译）"
fi

echo ""
echo "4. 检查可执行文件..."
if ros2 run motion_control --help 2>&1 | grep -q "grasp_planner"; then
    echo "  ✓ 可执行文件正常"
else
    echo "  ✗ 可执行文件异常"
fi

echo ""
echo "验证完成！"
```

保存为 `verify_member_c.sh`，然后运行：
```bash
chmod +x verify_member_c.sh
./verify_member_c.sh
```

---

## 📝 验证报告模板

验证完成后，填写以下报告：

```
成员C任务验证报告
==================

验证日期：___________
验证人：___________

1. 代码结构：✅/❌
2. 语法检查：✅/❌
3. 节点启动：✅/❌
4. Launch文件：✅/❌
5. 参数配置：✅/❌
6. 数据路径：✅/❌
7. 单臂功能：✅/❌
8. 双臂功能：✅/❌
9. 数据分析：✅/❌
10. 图表生成：✅/❌

总体评估：✅ 通过 / ❌ 需要修复

备注：
_________________________________
```

---

**最后更新**: 2024年

