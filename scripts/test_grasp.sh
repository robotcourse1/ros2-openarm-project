#!/bin/bash
# 抓取测试脚本

echo "OpenArm 抓取测试"
echo "================"
echo ""
echo "选择目标物体："
echo "1) 抓取苹果 (左侧)"
echo "2) 抓取香蕉 (右侧)"
echo "3) 自定义坐标"
echo ""
read -p "请输入选项 (1-3): " choice

case $choice in
    1)
        echo "发送抓取苹果的命令..."
        ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.85, y: -0.15, z: 0.79}" --once
        ;;
    2)
        echo "发送抓取香蕉的命令..."
        ros2 topic pub /target_pose geometry_msgs/Point "{x: 0.75, y: -0.15, z: 0.79}" --once
        ;;
    3)
        read -p "输入 x 坐标: " x
        read -p "输入 y 坐标: " y
        read -p "输入 z 坐标: " z
        echo "发送自定义坐标 ($x, $y, $z)..."
        ros2 topic pub /target_pose geometry_msgs/Point "{x: $x, y: $y, z: $z}" --once
        ;;
    *)
        echo "无效选项"
        exit 1
        ;;
esac

echo "命令已发送！"
