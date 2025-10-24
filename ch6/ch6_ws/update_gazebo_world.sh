#!/bin/bash

echo "🔄 Gazebo 世界更新工作流程"
echo "================================"

# 步骤1: 停止 Gazebo
echo "1️⃣ 停止 Gazebo..."
./stop_gazebo.sh

# 步骤2: 重新构建包
echo ""
echo "2️⃣ 重新构建包..."
colcon build --packages-select fishbot_description

if [ $? -eq 0 ]; then
    echo "✅ 构建成功"
else
    echo "❌ 构建失败"
    exit 1
fi

# 步骤3: 重新启动 Gazebo
echo ""
echo "3️⃣ 重新启动 Gazebo..."
echo "正在启动 Gazebo，请稍等..."
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py &

echo ""
echo "✅ 工作流程完成！"
echo "Gazebo 应该正在启动，请检查 GUI 窗口"
