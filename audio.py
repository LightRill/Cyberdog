#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
import time
import logging
import sys
import os

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("DIRECTION_TTS")

def setup_environment():
    """设置必要的ROS环境变量"""
    try:
        # 加载ROS环境
        env_setup = "source /opt/ros2/cyberdog/setup.bash"
        # 使用bash执行命令
        result = subprocess.Popen(
            "bash -c 'source /opt/ros2/cyberdog/setup.bash && echo 环境设置成功'",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        stdout, stderr = result.communicate()
        
        # 兼容Python 2和3的输出处理
        try:
            # Python 3
            output = stdout.decode('utf-8')
        except AttributeError:
            # Python 2
            output = stdout
        
        if '环境设置成功' in output:
            logger.info("环境设置成功")
            return True
        else:
            error_msg = stderr if stderr else "无错误信息"
            logger.error("环境设置失败: %s", error_msg)
            return False
    except Exception as e:
        logger.error("环境设置异常: %s", str(e))
        return False

def get_audio_node(retries=3, delay=1):
    """获取音频节点名（带重试机制）"""
    for i in range(retries):
        try:
            cmd = "ros2 node list | grep 'mi_' | head -n 1 | cut -d '/' -f 2"
            # 使用Popen捕获输出
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            stdout, stderr = process.communicate(timeout=10)
            
            # 兼容Python 2和3的输出处理
            try:
                # Python 3
                stdout_str = stdout.decode('utf-8')
            except AttributeError:
                # Python 2
                stdout_str = stdout
            
            # 检查返回码
            if process.returncode == 0:
                node_name = stdout_str.strip()
                if node_name:
                    logger.info("获取到音频节点: %s", node_name)
                    return node_name
                else:
                    logger.warning("节点名为空 (尝试 %d/%d)", i+1, retries)
            else:
                try:
                    # Python 3
                    error_msg = stderr.decode('utf-8', errors='ignore') if stderr else "未知错误"
                except AttributeError:
                    # Python 2
                    error_msg = stderr if stderr else "未知错误"
                logger.warning("命令执行失败 (尝试 %d/%d): %s", i+1, retries, error_msg)
        except subprocess.TimeoutExpired:
            logger.warning("命令执行超时 (尝试 %d/%d)", i+1, retries)
        except Exception as e:
            logger.warning("获取节点异常 (尝试 %d/%d): %s", i+1, retries, str(e))
        
        time.sleep(delay)
    
    logger.error("无法获取音频节点名，请检查语音服务状态")
    return None

def check_network():
    """检查网络连接状态"""
    try:
        # 测试连接到小米TTS服务器
        ping_cmd = "ping -c 2 api.mi.ai"
        process = subprocess.Popen(
            ping_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        _, stderr = process.communicate(timeout=10)
        
        if process.returncode == 0:
            logger.info("网络连接正常")
            return True
        else:
            try:
                # Python 3
                error_msg = stderr.decode('utf-8', errors='ignore') if stderr else "ping失败"
            except AttributeError:
                # Python 2
                error_msg = stderr if stderr else "ping失败"
            logger.warning("无法连接小米TTS服务: %s", error_msg)
            return False
    except Exception as e:
        logger.warning("网络检查异常: %s", str(e))
        return False

def play_direction(direction, node_name=None):
    """播放转向语音指令"""
    if direction not in ['left', 'right']:
        logger.error("无效方向参数: %s", direction)
        return False
    
    # 获取节点名（如果未提供）
    if not node_name:
        node_name = get_audio_node()
        if not node_name:
            return False
    
    # 映射方向到中文
    direction_text = {
        'left': '目标在左库区',
        'right': '目标在右库区'
    }[direction]
    
    # 构建ROS2命令
    pub_cmd = (
        "ros2 topic pub --once "
        "/%s/speech_play_extend "
        "protocol/msg/AudioPlayExtend "
        "'{\"is_online\": true, \"text\": \"%s\"}'" % (node_name, direction_text)
    )
    
    try:
        logger.info("播报方向: %s", direction_text)
        # 直接使用shell执行
        process = subprocess.Popen(
            pub_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        _, stderr = process.communicate(timeout=15)
        
        if process.returncode == 0:
            logger.info("播报命令执行成功")
            return True
        else:
            try:
                # Python 3
                error_msg = stderr.decode('utf-8', errors='ignore') if stderr else "未知错误"
            except AttributeError:
                # Python 2
                error_msg = stderr if stderr else "未知错误"
            logger.error("播报命令执行失败: %s", error_msg)
            return False
    except subprocess.TimeoutExpired:
        logger.error("播报命令执行超时")
    except Exception as e:
        logger.error("播报异常: %s", str(e))
    return False

def main():
    """主测试函数"""
    logger.info("=== 箭头方向语音播报测试 ===")
    
    # 设置环境
    if not setup_environment():
        sys.exit(1)
    
    # 检查网络
    check_network()
    
    # 获取音频节点
    node_name = get_audio_node()
    if not node_name:
        logger.error("无法获取音频节点，测试终止")
        sys.exit(1)
    
    # 测试左转
    logger.info("测试左转指令...")
    if play_direction('left', node_name):
        logger.info("左转指令发送成功")
    else:
        logger.error("左转指令发送失败")
    
    time.sleep(3)  # 等待播报完成
    
    # 测试右转
    logger.info("测试右转指令...")
    if play_direction('right', node_name):
        logger.info("右转指令发送成功")
    else:
        logger.error("右转指令发送失败")

if __name__ == '__main__':
    main()
    logger.info("测试结束")
