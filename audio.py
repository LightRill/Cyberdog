#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import rclpy
from rclpy.node import Node
from protocol.msg import AudioPlayExtend
import subprocess


def _detect_namespace() -> str:
    """自动检测 Cyberdog 命名空间（第一个 mi_ 开头的节点）"""
    try:
        cmd = "ros2 node list | grep 'mi_' | head -n 1 | cut -d '/' -f 2"
        ns = subprocess.check_output(cmd, shell=True).decode().strip()
        return ns
    except Exception:
        return ""


class AudioPublisher(Node):
    """专门负责语音播报的 ROS2 Publisher"""

    def __init__(self):
        super().__init__("audio_publisher")
        ns = _detect_namespace()
        topic = f"/{ns}/speech_play_extend" if ns else "/speech_play_extend"
        self.publisher = self.create_publisher(AudioPlayExtend, topic, 10)
        self.get_logger().info(f"✅ AudioPublisher 已绑定到 {topic}")

    def play_tts(self, text: str):
        msg = AudioPlayExtend()
        msg.is_online = True
        msg.text = text
        self.publisher.publish(msg)
        self.get_logger().info(f"🔊 播报: {text}")


# ---------- 对外接口 ----------
_audio_node = None
_audio_thread = None


def _init_audio_node():
    global _audio_node, _audio_thread
    if _audio_node is None:
        if not rclpy.ok():  # <= 先判断是否已初始化
            rclpy.init(args=None)
        _audio_node = AudioPublisher()
        _audio_thread = threading.Thread(
            target=rclpy.spin, args=(_audio_node,), daemon=True
        )
        _audio_thread.start()


def play_tts_async(text: str):
    """异步触发语音播报（不阻塞主程序）"""
    _init_audio_node()
    _audio_node.play_tts(text)


def shutdown_audio():
    """关闭音频节点"""
    global _audio_node, _audio_thread
    if _audio_node:
        _audio_node.destroy_node()
        _audio_node = None
    if rclpy.ok():
        rclpy.shutdown()
    if _audio_thread:
        _audio_thread.join(timeout=1.0)
        _audio_thread = None


# ---------- 测试 ----------
if __name__ == "__main__":
    play_tts_async("目标在左库区")
    import time

    time.sleep(2)  # 等 2 秒让消息发出去
    shutdown_audio()
