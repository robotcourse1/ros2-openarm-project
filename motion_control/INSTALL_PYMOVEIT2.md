# pymoveit2 安装指南

## 什么是pymoveit2？

pymoveit2是MoveIt2的Python接口库，提供了便捷的Python API来使用MoveIt2的运动规划功能。

## 安装方式选择

- **方式1**：直接在Ubuntu中git clone（需要网络）
- **方式2**：在Windows下载ZIP文件，然后复制到Ubuntu（推荐，适合网络不好时）
  - 详细步骤见：`INSTALL_PYMOVEIT2_WINDOWS.md`

## 安装步骤

### 步骤1：检查是否已存在

```bash
# 检查工作空间src目录
ls ~/openarm_ws/src/pymoveit2/

# 如果目录存在，说明已经克隆，直接跳到步骤3
```

### 步骤2：克隆pymoveit2仓库

```bash
# 进入工作空间src目录
cd ~/openarm_ws/src

# 克隆pymoveit2仓库
git clone https://github.com/JafarAbdi/pymoveit2.git

# 验证克隆成功
ls pymoveit2/
# 应该看到：CMakeLists.txt, package.xml, pymoveit2/ 等目录
```

### 步骤3：安装依赖

```bash
# 返回工作空间根目录
cd ~/openarm_ws

# 安装依赖（rosdep会自动检测并安装）
rosdep install --from-paths src --ignore-src -r -y

# 或者手动安装Python依赖（如果需要）
pip3 install numpy transforms3d
```

### 步骤4：编译pymoveit2

```bash
# 在工作空间根目录
cd ~/openarm_ws

# 编译pymoveit2包
colcon build --packages-select pymoveit2

# 如果编译成功，应该看到：
# Finished <<< pymoveit2 [X.XXs]
```

### 步骤5：配置环境

```bash
# source工作空间
source ~/openarm_ws/install/setup.bash

# 或者添加到 ~/.bashrc（永久配置）
echo "source ~/openarm_ws/install/setup.bash" >> ~/.bashrc
```

### 步骤6：验证安装

```bash
# 测试Python导入
python3 -c "from pymoveit2 import MoveIt2, GripperInterface; print('✓ pymoveit2安装成功！')"

# 如果成功，应该看到：
# ✓ pymoveit2安装成功！
```

## 完整安装命令（一键执行）

```bash
# 复制以下所有命令到终端执行

cd ~/openarm_ws/src

# 检查是否已存在
if [ -d "pymoveit2" ]; then
    echo "pymoveit2已存在，跳过克隆"
else
    echo "正在克隆pymoveit2..."
    git clone https://github.com/JafarAbdi/pymoveit2.git
fi

# 返回工作空间并安装依赖
cd ~/openarm_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
echo "正在编译pymoveit2..."
colcon build --packages-select pymoveit2

# 配置环境
source install/setup.bash

# 验证
echo "验证安装..."
python3 -c "from pymoveit2 import MoveIt2; print('✓ pymoveit2安装成功！')" || echo "✗ 安装失败，请检查错误信息"
```

## 验证安装成功

### 方法1：Python测试

```bash
python3 -c "from pymoveit2 import MoveIt2, GripperInterface; print('OK')"
```

### 方法2：运行节点测试

```bash
# 重新运行抓取规划节点
ros2 run motion_control grasp_planner

# 应该不再看到 "pymoveit2 not found" 的警告
# 应该看到正常的启动信息
```

### 方法3：检查包

```bash
# 检查pymoveit2包是否在ROS2包列表中
ros2 pkg list | grep pymoveit2

# 应该输出：pymoveit2
```

## 常见问题

### 问题1：git clone失败（网络问题）

**解决方法**：
```bash
# 使用镜像或代理
# 或者手动下载zip文件解压到 ~/openarm_ws/src/pymoveit2
```

### 问题2：编译错误

**检查**：
```bash
# 检查依赖是否安装
rosdep check --from-paths src --ignore-src

# 检查CMakeLists.txt
cat ~/openarm_ws/src/pymoveit2/CMakeLists.txt
```

### 问题3：导入失败

**检查**：
```bash
# 确认已source环境
source ~/openarm_ws/install/setup.bash

# 检查Python路径
python3 -c "import sys; print('\n'.join(sys.path))" | grep openarm_ws

# 应该看到工作空间的install目录
```

### 问题4：找不到包

**解决**：
```bash
# 重新编译并source
cd ~/openarm_ws
colcon build --packages-select pymoveit2
source install/setup.bash

# 检查安装
ls ~/openarm_ws/install/pymoveit2/
```

## 安装后测试

### 测试1：重新运行节点

```bash
cd ~/openarm_ws
source install/setup.bash
ros2 run motion_control grasp_planner

# 应该看到：
# [INFO] [grasp_planner]: 抓取规划节点已启动...
# 不再有 "pymoveit2 not found" 警告
```

### 测试2：测试MoveIt2功能

```bash
# 确保MoveIt2服务运行
# 然后运行节点，应该可以正常使用运动规划功能
```

## 快速安装脚本

创建一个安装脚本 `install_pymoveit2.sh`：

```bash
#!/bin/bash
set -e

echo "=========================================="
echo "pymoveit2 安装脚本"
echo "=========================================="

WS_DIR="$HOME/openarm_ws"
SRC_DIR="$WS_DIR/src"

cd "$SRC_DIR"

# 检查是否已存在
if [ -d "pymoveit2" ]; then
    echo "✓ pymoveit2已存在，跳过克隆"
else
    echo "正在克隆pymoveit2..."
    git clone https://github.com/JafarAbdi/pymoveit2.git
    echo "✓ 克隆完成"
fi

# 安装依赖
echo "正在安装依赖..."
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y

# 编译
echo "正在编译pymoveit2..."
colcon build --packages-select pymoveit2

# 配置环境
source install/setup.bash

# 验证
echo "验证安装..."
if python3 -c "from pymoveit2 import MoveIt2" 2>/dev/null; then
    echo "✓ pymoveit2安装成功！"
    exit 0
else
    echo "✗ 安装失败，请检查错误信息"
    exit 1
fi
```

使用方法：
```bash
chmod +x install_pymoveit2.sh
./install_pymoveit2.sh
```

---

**安装完成后**，重新运行节点就不会看到警告了！
