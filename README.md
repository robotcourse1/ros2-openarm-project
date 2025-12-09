\# ✅ \*\*README.md（纯 Markdown 文件，直接可用）\*\*



```markdown

\# ROS2 + OpenArm 双臂系统：视觉感知 · 手眼标定 · 运动规划 · 抓取系统



本项目基于 \*\*ROS2 (Humble/Foxy)\*\*、\*\*OpenArm 双臂机器人\*\*、\*\*MuJoCo/Gazebo 仿真环境\*\* 与 \*\*深度相机\*\*，实现从  

\*\*物体检测 → 三维定位 → 手眼标定 → 双臂轨迹规划 → 抓取执行\*\* 的完整智能机器人系统。



---



\## 📘 项目简介



本系统实现以下完整闭环流程：



1\. 深度相机采集 RGB/Depth 数据  

2\. 图像/点云预处理  

3\. 物体检测（传统方法 + 深度模型支持）  

4\. 目标三维定位（坐标变换 camera → base）  

5\. 手眼标定（Tsai-Lenz / AX = XB）  

6\. MoveIt2 运动规划（双臂协同工作）  

7\. 夹爪控制与抓取状态机  

8\. 多轮抓取实验统计与误差分析  

9\. MuJoCo 仿真与实物平台对比验证（若具备实物设备）



---



\## 🏗 系统结构图



（可替换为 `docs/system\_architecture.png`）



```



相机 → 感知模块 → 物体3D定位 → 手眼标定 → 运动规划器 → 控制器 → 夹爪执行



````



---



\## 🔧 功能模块



| 模块 | 功能说明 |

|------|----------|

| \*\*模型与仿真（成员2）\*\* | URDF/Xacro 扩展、相机模型、MuJoCo 场景构建 |

| \*\*视觉感知（成员3）\*\* | 图像/深度处理、物体检测、聚类、三维定位 |

| \*\*手眼标定（成员4）\*\* | 标定数据采集、AX=XB 求解、误差评估 |

| \*\*运动规划（成员4）\*\* | MoveIt2 轨迹规划、双臂协作策略 |

| \*\*抓取执行（成员4）\*\* | 夹爪控制、抓取状态机、成功率统计 |

| \*\*系统集成（成员1）\*\* | 接口设计、TF 框架、总控 launch、视频录制 |



---



\## 📦 环境依赖



\### 必要软件

\- ROS2 Humble / Foxy

\- Python ≥ 3.8

\- MoveIt2

\- MuJoCo 2.x / mujoco\_ros / ros\_gz

\- PCL / OpenCV



\### 安装命令示例



```bash

sudo apt install ros-${ROS\_DISTRO}-vision-msgs ros-${ROS\_DISTRO}-pcl-ros

pip install opencv-python numpy transforms3d

````



---



\## ⚙ 安装与构建



```bash

mkdir -p ~/ros2\_openarm\_ws/src

cd ~/ros2\_openarm\_ws/src

git clone <本仓库地址>

cd ..

rosdep install --from-paths src -r -y --ignore-src

colcon build

source install/setup.bash

```



---



\## 🚀 系统运行说明



\### ▶ 启动完整仿真系统



```bash

ros2 launch system\_bringup mujoco\_full\_system.launch.py

```



启动内容包含：



\* OpenArm 双臂机器人

\* RGBD 相机

\* 桌子与物体模型

\* 感知节点

\* 手眼标定模块

\* 运动规划和控制器



---



\### ▶ 单独运行视觉感知节点



```bash

ros2 launch perception perception.launch.py

```



查看检测结果：



```bash

ros2 topic echo /detected\_objects

```



---



\### ▶ 手眼标定流程



采集样本：



```bash

ros2 run calibration collect\_samples

```



计算标定矩阵：



```bash

ros2 run calibration compute\_hand\_eye

```



结果输出：



\* `results/calibration\_result.yaml`

\* `results/calibration\_error.csv`



---



\### ▶ 抓取规划与执行



```bash

ros2 launch motion\_control grasp.launch.py

```



抓取状态机流程：



```

APPROACH → PREGRASP → CLOSE\_GRIPPER → LIFT → RETRACT

```



---



\## 📁 项目目录结构



```text

.

├── launch/

│   ├── mujoco\_full\_system.launch.py

│   ├── perception.launch.py

│   ├── calibration.launch.py

│   └── motion\_control.launch.py

│

├── urdf/

│   ├── openarm\_with\_camera.xacro

│   ├── apple.xacro

│   ├── banana.xacro

│   └── table.xacro

│

├── mujoco/

│   └── mujoco\_world.xml

│

├── perception/

│   ├── src/object\_detector.py

│   ├── src/pointcloud\_processing.py

│   └── config/perception\_params.yaml

│

├── calibration/

│   ├── scripts/collect\_samples.py

│   └── scripts/compute\_hand\_eye.py

│

├── motion\_control/

│   ├── src/motion\_planner.py

│   ├── src/gripper\_controller.py

│   └── src/grasp\_state\_machine.py

│

├── docs/

│   ├── system\_architecture.pdf

│   ├── interfaces.md

│   └── hardware\_setup.pdf

│

├── results/

│   ├── perception\_accuracy.csv

│   ├── grasp\_success.csv

│   └── calibration\_error.csv

│

└── README.md

```



---



\## 🎬 成果展示（可放图片或视频）



示例占位符：



\* 🍎 物体检测效果图

\* 🤖 抓取演示截图

\* 📈 标定误差曲线

\* 📊 抓取成功率统计



（建议将资源放在 `media/` 目录）



---



\## 👥 团队成员与详细分工



\### \*\*成员1 — 系统架构与集成\*\*



\* 系统架构设计、ROS2 接口定义、TF 框架

\* 总控 launch、仿真整合、论文统稿



\### \*\*成员2 — 机械模型与仿真\*\*



\* URDF/Xacro 扩展、相机模型、桌面/物体模型

\* MuJoCo 场景配置、相机参数/噪声建模



\### \*\*成员3 — 视觉感知与定位\*\*



\* 图像/深度处理、物体检测、三维定位

\* 定位精度评估实验与可视化



\### \*\*成员4 — 手眼标定与规划抓取\*\*



\* AX=XB 标定、误差分析

\* MoveIt2 规划、夹爪控制、抓取状态机

\* 抓取成功率实验



---



\## 📚 参考文献



1\. ROS2 Documentation

2\. MoveIt2 Documentation

3\. MuJoCo Documentation

4\. Tsai R. \& Lenz R., “A New Technique for Hand-Eye Calibration”

5\. PCL / OpenCV 官方文档



---



\## 📝 License



MIT License. 可在科研、教学与学习场景中自由使用与修改。



```



