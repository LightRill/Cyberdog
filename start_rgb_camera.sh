#!/bin/bash

# 定义退出函数
cleanup() {
    echo "Stopping nodes..."
    kill $REALSENSE_PID $STEREO_PID 2>/dev/null
    exit 0
}

# 捕获 CTRL+C 信号
trap cleanup SIGINT

# 启动 realsense 相机节点
echo "Launching realsense2_camera..."
ros2 launch realsense2_camera on_dog.py &
REALSENSE_PID=$!

# 等待相机节点启动
sleep 5

# 配置并激活 realsense 相机
echo "Configuring /camera/camera..."
ros2 lifecycle set /camera/camera configure
echo "Activating /camera/camera..."
ros2 lifecycle set /camera/camera activate

# 启动立体相机测试节点
echo "Launching stereo_camera..."
ros2 launch camera_test stereo_camera.py &
STEREO_PID=$!

# 等待立体相机节点启动
sleep 5

# 配置并激活立体相机
echo "Configuring /stereo_camera..."
ros2 lifecycle set /stereo_camera configure
echo "Activating /stereo_camera..."
ros2 lifecycle set /stereo_camera activate

echo "All nodes launched and activated."

# 等待子进程，保持脚本运行，支持 CTRL+C 退出
wait $REALSENSE_PID $STEREO_PID
