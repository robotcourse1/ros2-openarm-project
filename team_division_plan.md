团队分工与任务计划

## 粗略分工表（均衡+资源指引）
- A 环境与仿真工程师  
  - 资源：`openarm_description`（URDF/Xacro/meshes）、`openarm_ros2/openarm_bringup`（launch）、`openarm_moveit_config`（URDF生成链）；自建 MuJoCo/Gazebo 配置。  
- B 视觉与手眼标定工程师  
  - 资源：`openarm_description`（相机挂载/TF）、`openarm_ros2/openarm_bringup`（TF/launch）、`openarm_moveit_config/config/sensors_3d.yaml`（点云接入 MoveIt）；标定脚本自建。  
- C 运动规划与控制工程师  
  - 资源：`openarm_moveit_config`（SRDF/规划器/控制器）、`openarm_ros2/openarm_bringup`（控制器 spawner）、`openarm_control`（动作客户端示例）；可用 `ompl` 库做规划对比。  
- D 系统集成与论文统稿  
  - 资源：`openarm_ros2/openarm_bringup`（集成 launch）、`openarm_control`（控制示例）、全局实验数据；负责排版、视频与提交材料。  

## 详细任务与实现要点

### 成员 A：环境与仿真建设
- 扩展 URDF/Xacro：在 `openarm_description/urdf/robot/openarm_robot.xacro` 挂桌子、苹果/香蕉模型；为 B 预留相机安装位（link+fixed joint、惯性占位）。
- 仿真场景：编写 MuJoCo 或 Gazebo/ros_gz 配置，加载机器人+桌面+物体；参考 `openarm_ros2/openarm_bringup` 模板新建/修改 launch。
- TF 管理：核对 world/base/camera/ee/target TF 连续性，提供 RViz 截图。
- 交付：场景 URDF/XML、launch、RViz/MuJoCo 截图；论文“硬件模型扩展”“仿真环境搭建”素材。

### 成员 B：视觉感知与手眼标定
- 相机集成：在 A 预留位添加深度相机 link/joint，生成 TF；必要时写内参/噪声占位。
- 标定：选“眼在手上/外”，用 eye_hand_calibration 或自写脚本多姿态采样，输出外参与误差表，保存数据与脚本。
- 检测与坐标变换：实现颜色/形状或轻量模型检测；完成 camera_frame → base_frame 转换并验证精度。
- 交付：标定脚本+误差报告、检测节点、定位误差与截图；论文“手眼标定”“视觉检测与定位”素材。

### 成员 C：运动规划与控制
- MoveIt 配置与规划：基于 `openarm_moveit_config` 选/调 OMPL 规划器，设约束与时间参数化。
- 抓取流程：生成预抓取/抓取/撤离姿态；用 `openarm_control/arm_controller.py` 或 MoveIt action 执行轨迹，增加夹爪开合逻辑。
- 双臂策略：若用双臂，结合 `openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py` 的命名空间与控制器；不稳定时先单臂。
- 交付：抓取/控制节点、轨迹平滑与成功率数据、曲线图；论文“抓取规划与控制”与“实验结果”素材。

### 成员 D：系统集成、实验与论文统稿
- 集成与 Launch：编写总控 launch，将相机/标定/感知/规划/控制串联；绘制系统框图（Topic/Service/TF/数据流）。
- 实验管理：统一数据表（标定误差、定位误差、抓取成功率），收集截图/视频。
- 仓库与流程：负责 GitHub 仓库（如 `ros2-openarm-project.git`）管理与分支策略，建立 README/CI（可选）、提交规范，合并各成员成果。
- 论文与提交：撰写摘要/引言/实验分析/总结，合并 A/B/C 章节；统一格式、图表编号、参考文献；制作 ≤8 分钟演示视频（拍摄/录屏/剪辑）与旁白脚本；整理代码仓库与提交包。
- 交付：总控 launch、系统框图、实验表格与统计图、最终 PDF、视频与链接、分工声明、仓库发布（tag/release）。

## 协同与节奏
- 阶段1：A 完成场景+相机位；B 初版标定；C 基线 MoveIt 规划；D 起草框图与文档框架。
- 阶段2：B 完成检测与坐标变换；C 接入坐标跑抓取闭环；A 提供稳定仿真与物体模型；D 汇总实验表格。
- 阶段3：优化精度/成功率（滤波、规划器、限速/力控），补足截图视频；D 统稿排版、参考文献与附录。
- 加分点：仿真 vs（若有）实物误差对比；定位/抓取成功率 ≥ 90%；附录含核心脚本与复现实验步骤；视频展示标定、检测、抓取全过程。

