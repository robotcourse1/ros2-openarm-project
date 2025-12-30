#!/bin/bash
# 诊断可执行文件问题

echo "=========================================="
echo "可执行文件诊断脚本"
echo "=========================================="
echo ""

cd ~/openarm_ws
source install/setup.bash 2>/dev/null

echo "1. 检查可执行文件是否存在..."
echo "----------------------------------------"
EXECUTABLES=("grasp_planner" "bimanual_grasp_planner" "statistics_analyzer" "gripper_controller" "test_grasp")

for exec_name in "${EXECUTABLES[@]}"; do
    # 检查安装目录中的文件
    INSTALL_PATH="$HOME/openarm_ws/install/motion_control/lib/motion_control/$exec_name"
    if [ -f "$INSTALL_PATH" ]; then
        echo "  ✓ $exec_name 存在: $INSTALL_PATH"
        # 检查是否可执行
        if [ -x "$INSTALL_PATH" ]; then
            echo "    ✓ 可执行权限正常"
        else
            echo "    ✗ 缺少执行权限"
            chmod +x "$INSTALL_PATH"
            echo "    ✓ 已修复执行权限"
        fi
    else
        echo "  ✗ $exec_name 不存在: $INSTALL_PATH"
    fi
done

echo ""
echo "2. 测试可执行文件..."
echo "----------------------------------------"
for exec_name in "${EXECUTABLES[@]}"; do
    echo "测试: $exec_name"
    timeout 2 ros2 run motion_control "$exec_name" --help 2>&1 | head -3
    if [ ${PIPESTATUS[0]} -eq 0 ] || [ ${PIPESTATUS[0]} -eq 124 ]; then
        echo "  ✓ $exec_name 可以运行"
    else
        echo "  ✗ $exec_name 运行异常"
    fi
    echo ""
done

echo ""
echo "3. 检查entry_points配置..."
echo "----------------------------------------"
cd ~/openarm_ws/src/ros2-openarm-project/motion_control
grep -A 10 "entry_points" setup.py | grep "="

echo ""
echo "4. 建议..."
echo "----------------------------------------"
echo "如果可执行文件有问题，运行："
echo "  cd ~/openarm_ws"
echo "  colcon build --packages-select motion_control"
echo "  source install/setup.bash"
echo ""

