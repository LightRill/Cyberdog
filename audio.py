#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import subprocess


def play_tts(text: str) -> bool:
    """
    播放语音（ROS2 topic 发布方式）
    输入: text (字符串)
    返回: True(成功) / False(失败)
    """
    try:
        # 获取音频节点
        cmd_node = "ros2 node list | grep 'mi_' | head -n 1 | cut -d '/' -f 2"
        node_name = (
            subprocess.check_output(cmd_node, shell=True).decode("utf-8").strip()
        )
        if not node_name:
            return False

        # 构建发布命令
        pub_cmd = (
            "ros2 topic pub --once "
            f"/{node_name}/speech_play_extend "
            "protocol/msg/AudioPlayExtend "
            f"'" + '{"is_online": true, "text": "' + text + '"}' + "'"
        )

        # 执行
        result = subprocess.run(
            pub_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
        )

        return result.returncode == 0
    except Exception:
        return False


if __name__ == "__main__":
    if play_tts("目标在左库区"):
        print("✅ 播报成功")
    else:
        print("❌ 播报失败")
