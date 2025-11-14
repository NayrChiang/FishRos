#!/bin/bash

echo "🔄 Gazebo World Update Workflow"
echo "================================"

# Step 1: Stop Gazebo
echo "1️⃣ Stopping Gazebo..."
./stop_gazebo.sh

# Step 2: Rebuild package
echo ""
echo "2️⃣ Rebuilding package..."
colcon build --packages-select fishbot_description

if [ $? -eq 0 ]; then
    echo "✅ Build successful"
else
    echo "❌ Build failed"
    exit 1
fi

# Step 3: Restart Gazebo
echo ""
echo "3️⃣ Restarting Gazebo..."
echo "Starting Gazebo, please wait..."
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py &

echo ""
echo "✅ Workflow completed!"
echo "Gazebo should be starting, please check the GUI window"
