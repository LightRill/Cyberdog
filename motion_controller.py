#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import time
import subprocess

# 导入 ROS2 消息类型
from protocol.msg import MotionServoCmd


def get_namespace():
    """自动检测 ROS2 命名空间"""
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
        )
        for line in result.stdout.splitlines():
            if "mi_" in line:
                parts = line.strip().split("/")
                if len(parts) > 1:
                    return "/" + parts[1]
    except subprocess.CalledProcessError as e:
        print("获取命名空间失败:", e)
    return "/"  # 默认根命名空间


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller_ros2", namespace=get_namespace())

        # 创建 ROS2 发布者
        self.pub = self.create_publisher(MotionServoCmd, "motion_servo_cmd", 10)

        # 状态变量
        self.running = True
        self.send_lock = threading.Lock()

        # 初始化控制消息
        self.cmd_msg = MotionServoCmd()
        self.init_cmd_msg()

        # 启动线程，并保存引用
        self.send_thread = threading.Thread(target=self.send_publish)
        self.send_thread.daemon = True
        self.send_thread.start()

        self.get_logger().info("初始化完成：循环自动发送已启动")

    def init_cmd_msg(self):
        """初始化 ROS2 控制指令"""
        self.cmd_msg.motion_id = 111  # 默认站立
        self.cmd_msg.cmd_type = MotionServoCmd.SERVO_DATA
        self.cmd_msg.cmd_source = MotionServoCmd.VIS
        self.cmd_msg.value = 0
        self.cmd_msg.vel_des = [0.0, 0.0, 0.0]
        self.cmd_msg.step_height = [0.0, 0.0]

    def control_run(self):
        """启动后台线程，循环发送控制消息"""
        send_thread = threading.Thread(target=self.send_publish)
        send_thread.daemon = True
        send_thread.start()

    def send_publish(self):
        while self.running and rclpy.ok():
            try:
                with self.send_lock:
                    self.pub.publish(self.cmd_msg)
            except rclpy._rclpy_pybind11.RCLError:
                # 退出线程，不抛异常
                break
            time.sleep(0.05)

    def stop_thread(self):
        """安全停止后台线程"""
        self.running = False
        if self.send_thread.is_alive():
            self.send_thread.join()

    def wait_stand(self):
        self.get_logger().info("等待机器人站立")
        with self.send_lock:
            self.cmd_msg.motion_id = 111
            self.cmd_msg.vel_des = [0.0, 0.0, 0.0]
        time.sleep(2.0)
        self.get_logger().info("站立完成")

    def forward(self, speed=0.2):
        with self.send_lock:
            self.cmd_msg.motion_id = 308
            self.cmd_msg.vel_des = [speed, 0.0, 0.0]
            self.cmd_msg.step_height = [0.05, 0.05]
        self.get_logger().info(f"前进中，速度 {speed} m/s")

    def stop(self):
        with self.send_lock:
            self.cmd_msg.motion_id = 101
            self.cmd_msg.vel_des = [0.0, 0.0, 0.0]
        self.get_logger().info("已发送停止指令")


def main():
    rclpy.init()
    controller = MotionController()

    try:
        controller.wait_stand()
        time.sleep(3)

        controller.forward(0.2)
        time.sleep(5)

        controller.stop()
        time.sleep(2)

    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False
        controller.stop_thread()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
