# OpenArm 环境与仿真（分工 A 交付验证指南）

本指南只覆盖环境/仿真部分（桌子+苹果/香蕉+相机+机器人姿态）。不改动项目根 README。

## 1. 前提
- 已安装 ROS 2 Humble，`colcon`，`xacro`
- 推荐先安装以下依赖：
```bash
# ros2_control 相关（如果编译 openarm_hardware 需要）
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# graphviz（用于生成 TF 树 PDF 图）
sudo apt install graphviz
```

## 2. 进入正确工作空间
确保源码位于 `~/openarm_ws/src`（含 `openarm_description`、`openarm_env_description`、`openarm_env_bringup`）。所有构建命令都在 `~/openarm_ws` 根目录执行。

## 3. 构建与环境
```bash
cd ~/openarm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select openarm_description openarm_env_description openarm_env_bringup
source install/setup.bash
```
若出现 “ignoring unknown package ...”，说明当前目录不是含 `src` 的工作空间根，或源码不在 `src/`。请确认后重试。

## 4. 快速验证清单（可截图留存）

**重要提示**：以下验证步骤中，4.2 和 4.3 的测试需要在不同的终端进行。所有新终端都需要先执行 `cd ~/openarm_ws && source install/setup.bash`。

### 4.1 Xacro 解析
```bash
xacro $(ros2 pkg prefix openarm_env_description)/share/openarm_env_description/urdf/openarm_with_env.xacro > /tmp/test.urdf
wc -l /tmp/test.urdf    # 预期约 590 行
```

### 4.2 RViz 场景
终端 1：
```bash
ros2 launch openarm_env_bringup display_env.launch.py
```
终端 2（检查 robot_description）：
```bash
ros2 topic echo /robot_description --once | head -20
```
终端 3（检查关键 TF）：
```bash
ros2 run tf2_ros tf2_echo world table
ros2 run tf2_ros tf2_echo world apple
ros2 run tf2_ros tf2_echo world depth_camera_link
```
终端 4（生成 TF 树图，可存档）：
**重要：必须先确保终端 1 的 `display_env.launch.py` 正在运行，否则 `view_frames` 会返回空的 TF 树。**

先检查 TF 是否在发布：
```bash
ros2 topic echo /tf_static --once | head -10
ros2 topic list | grep tf
```

如果看到 `/tf` 和 `/tf_static` 话题，再运行：
```bash
ros2 run tf2_tools view_frames
# 等待 5 秒后，检查当前目录是否生成了 frames.pdf
ls -lh frames.pdf
# 如果文件存在，再打开
evince frames.pdf
```

**注意**：如果 `view_frames` 输出 `frame_yaml='[]'`，说明没有检测到 TF 数据，请确保 `display_env.launch.py` 在另一个终端运行中。

### 4.3 MuJoCo 场景
```bash
ros2 launch openarm_env_bringup mujoco_env.launch.py
```
说明：当前机器人在 MuJoCo 中为占位方块，桌面/水果位置与 URDF 对齐。

## 5. 关键文件（供审阅）
- `openarm_env_description/urdf/openarm_with_env.xacro`：机器人、桌子、苹果、香蕉、相机的位姿配置。
- `openarm_env_description/mujoco/openarm_env.xml`：MuJoCo 场景坐标与 URDF 对齐。
- `openarm_env_bringup/launch/display_env.launch.py`：RViz 启动。
- `openarm_env_bringup/launch/mujoco_env.launch.py`：MuJoCo viewer 启动。
- `openarm_env_bringup/rviz/openarm_env.rviz`：RViz 配置（隐藏多余 TF 坐标轴）。

## 6. 常见问题
- **找不到包**：确认在 `~/openarm_ws` 运行 `colcon build` 且已 `source install/setup.bash`；检查 `ros2 pkg prefix openarm_env_description` 输出应指向 `~/openarm_ws/install/...`
- **TF 报 frame 不存在**：确保 `display_env.launch.py` 已启动；用 `ros2 node list` 查看 `robot_state_publisher` 是否在跑。
- **`view_frames` 返回 `frame_yaml='[]'`**：说明没有检测到 TF 数据。必须先启动 `display_env.launch.py` 并保持运行，然后在另一个终端运行 `view_frames`。检查方法：
  ```bash
  ros2 topic list | grep tf  # 应该看到 /tf 和 /tf_static
  ros2 topic echo /tf_static --once | head -5  # 应该能看到TF数据
  ```
- **`frames.pdf` 找不到**：
  - `view_frames` 会在**当前工作目录**生成 `frames.pdf`。如果运行命令时在 `~/openarm_ws`，文件就在那里；如果在 `~/桌面`，文件就在那里。
  - 如果 `view_frames` 报告 "Generating graph in frames.pdf file" 但文件不存在：
    1. **检查是否有错误输出**：`view_frames` 可能静默失败，检查完整输出中是否有警告或错误
    2. **查找文件**：`find ~ -name frames.pdf 2>/dev/null` 或 `find /tmp -name frames.pdf 2>/dev/null`
    3. **检查 graphviz 是否正常工作**：`dot -V`（应该显示版本信息）
    4. **替代方案**：如果 PDF 确实无法生成，`view_frames` 输出的 `frame_yaml` 字段已经包含了完整的 TF 树信息，可以作为验证依据。TF 树结构如下：
       ```
       world
       ├── table
       │   ├── apple
       │   └── banana
       └── openarm_body_link0
           ├── openarm_left_link0 → ... → openarm_left_link7
           ├── openarm_right_link0 → ... → openarm_right_link7
           └── depth_camera_link
               └── depth_camera_optical_frame
       ```
       只要 `view_frames` 成功输出了 `frame_yaml`（非空），就说明 TF 树配置正确。
- **MuJoCo 不显示物体**：确认 `mujoco_env.xml` 路径未改，必要时重新 `colcon build --symlink-install` 并 source。

## 7. 交付建议
- **必须提交**：
  - RViz 场景截图（显示机器人、桌子、苹果、香蕉正确位置）
  - MuJoCo 场景截图（显示场景配置）
  - `/robot_description` 话题输出片段（证明 URDF 正确加载）
- **TF 树验证**：
  - 如果 `view_frames` 成功生成了 `frames.pdf`，提交该 PDF
  - 如果 PDF 未生成但 `view_frames` 输出了非空的 `frame_yaml`，可以提交该输出作为验证（说明 TF 树配置正确）
  - 关键验证点：`world` → `table` → `{apple, banana}` 和 `world` → `openarm_body_link0` → `{arms, camera}` 的父子关系正确
- **可选优化**：如需真实机器人网格到 MuJoCo，可在 `openarm_env_description/mujoco/openarm_env.xml` 替换占位 geom。





## 附录（测试运行命令）

#### ========== 1. 环境准备与构建 ==========
cd ~/openarm_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select openarm_description openarm_env_description openarm_env_bringup
source install/setup.bash

#### ========== 2. 验证包安装 ==========
ros2 pkg list | grep openarm
ls install/openarm_env_bringup/share/openarm_env_bringup/launch

#### ========== 3. 测试 Xacro 解析 ==========
xacro /home/ljh/openarm_ws/install/openarm_env_description/share/openarm_env_description/urdf/openarm_with_env.xacro > /tmp/test.urdf
wc -l /tmp/test.urdf

#### ========== 4. RViz 场景测试 ==========
##### 终端1：启动RViz（必须保持运行）
ros2 launch openarm_env_bringup display_env.launch.py
##### 保持此终端运行，不要关闭

##### 终端2：检查robot_description话题（新终端，需先 source）
cd ~/openarm_ws
source install/setup.bash
ros2 topic echo /robot_description --once | head -20

##### 终端3：检查TF树（新终端，需先 source）
cd ~/openarm_ws
source install/setup.bash
##### 等待终端1的launch完全启动后（约3-5秒），再运行以下命令
ros2 run tf2_ros tf2_echo world table
ros2 run tf2_ros tf2_echo world apple
ros2 run tf2_ros tf2_echo world depth_camera_link

##### 终端4：生成TF树图（新终端，需先 source，且终端1必须正在运行）
cd ~/openarm_ws
source install/setup.bash
##### 先确认TF话题存在
ros2 topic list | grep tf
##### 如果看到 /tf 和 /tf_static，再运行
##### 注意：view_frames 需要 graphviz 来生成 PDF，如果没有安装会失败
##### 如果未安装，先运行：sudo apt install graphviz
ros2 run tf2_tools view_frames
##### 等待5秒后，检查文件是否生成（view_frames 会在当前工作目录生成 frames.pdf）
pwd  # 查看当前目录
ls -lh frames.pdf  # 检查文件是否存在
##### 如果文件存在，再打开
if [ -f frames.pdf ]; then
    evince frames.pdf
else
    echo "注意：frames.pdf 未在当前目录生成。"
    echo "但 view_frames 已成功收集 TF 数据（见上方的 frame_yaml 输出）。"
    echo ""
    echo "TF 树验证："
    echo "- 如果 frame_yaml 不为空，说明 TF 树配置正确"
    echo "- 主要框架关系：world → table → {apple, banana}; world → openarm_body_link0 → {arms, camera}"
    echo ""
    echo "如果确实需要 PDF："
    echo "1. 检查 graphviz：dot -V"
    echo "2. 查找文件：find ~ -name frames.pdf 2>/dev/null"
    echo "3. 检查 view_frames 的完整输出是否有错误"
fi

#### ========== 5. MuJoCo 场景测试 ==========
##### 终端1：启动MuJoCo viewer
ros2 launch openarm_env_bringup mujoco_env.launch.py

#### ========== 6. 检查话题列表（需先启动终端1的launch）==========
##### 新终端，需先 source
cd ~/openarm_ws
source install/setup.bash

##### 确保终端1的 display_env.launch.py 正在运行
ros2 topic list
ros2 topic echo /joint_states --once

#### ========== 7. 验证所有TF连接（需先启动终端1的launch）==========
##### 新终端，需先 source
cd ~/openarm_ws
source install/setup.bash

##### 确保终端1的 display_env.launch.py 正在运行，等待3-5秒让TF完全初始化
ros2 run tf2_ros tf2_echo world openarm_body_link0
ros2 run tf2_ros tf2_echo world banana
ros2 run tf2_ros tf2_echo openarm_body_link0 depth_camera_link
ros2 run tf2_ros tf2_echo depth_camera_link depth_camera_optical_frame
