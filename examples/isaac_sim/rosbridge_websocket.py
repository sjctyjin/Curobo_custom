#!/usr/bin/env python3
# rosbridge_websocket.py - ROS Bridge WebSocket通信模块

import json
import time
import threading
import numpy as np

try:
    import websocket
    import ssl
except ImportError:
    raise ImportError("请先安装WebSocket客户端库: pip install websocket-client")

class ROSBridgeClient:
    """ROSBridge WebSocket客户端，用于向ROS系统发送消息"""
    
    def __init__(self, host='127.0.0.1', port=9090, use_ssl=False):
        """
        初始化ROSBridge WebSocket客户端
        Args:
            host: ROSBridge服务器主机
            port: ROSBridge服务器端口（通常为9090）
            use_ssl: 是否使用SSL连接
        """
        self.host = host
        self.port = port
        self.use_ssl = use_ssl
        self.ws = None
        self.connected = False
        self.message_counter = 0
        self._lock = threading.Lock()
        
        # 构建WebSocket URL
        protocol = "wss" if use_ssl else "ws"
        self.url = f"{protocol}://{host}:{port}"
        
        # 连接到ROSBridge
        self._connect()
    
    def _connect(self):
        """连接到ROSBridge WebSocket服务器"""
        try:
            # 配置WebSocket
            websocket.enableTrace(False)
            if self.use_ssl:
                self.ws = websocket.create_connection(
                    self.url,
                    sslopt={"cert_reqs": ssl.CERT_NONE}
                )
            else:
                self.ws = websocket.create_connection(self.url)
            
            self.connected = True
            print(f"已连接到ROSBridge WebSocket服务器: {self.url}")
            return True
        except Exception as e:
            print(f"连接到ROSBridge失败: {e}")
            self.connected = False
            return False
    
    def publish(self, topic, msg_type, msg):
        """
        发布消息到ROS话题
        Args:
            topic: ROS话题名称（如'/joint_states'）
            msg_type: 消息类型（如'sensor_msgs/JointState'）
            msg: 消息内容（字典格式）
        """
        with self._lock:
            if not self.connected:
                if not self._connect():
                    print("未连接，无法发布消息")
                    return False
            
            try:
                # 增加消息计数
                self.message_counter += 1
                
                # 构建ROSBridge协议消息
                rosbridge_msg = {
                    "op": "publish",
                    "topic": topic,
                    "type": msg_type,
                    "msg": msg
                }
                
                # 发送消息
                self.ws.send(json.dumps(rosbridge_msg))
                
                # 每500条消息打印一次状态
                if self.message_counter % 500 == 0:
                    print(f"已发送 {self.message_counter} 条消息到 {topic}")
                
                return True
            except Exception as e:
                print(f"发送消息时出错: {e}")
                self.connected = False
                return False
    
    def publish_joint_state(self, topic, joint_positions, joint_names, frame_id="piper_single"):
        """
        发布JointState关节状态到ROS话题
        Args:
            topic: ROS话题名称 (通常为'/joint_states')
            joint_positions: 关节位置值（NumPy数组或列表）
            joint_names: 关节名称列表
            frame_id: header中的frame_id字段
        """
        # 确保joint_names和joint_positions不为空
        if not joint_names or len(joint_positions) == 0:
            print("关节名称或位置为空，无法发送")
            return False
        
        # 确保joint_positions是列表
        if isinstance(joint_positions, np.ndarray):
            positions_list = joint_positions.tolist()
        else:
            positions_list = joint_positions
        
        # 确保数组长度匹配
        if len(joint_names) != len(positions_list):
            print(f"警告: 关节名称数量({len(joint_names)})与位置数量({len(positions_list)})不匹配")
            
            # 取两者中较小的长度
            min_length = min(len(joint_names), len(positions_list))
            joint_names = joint_names[:min_length]
            positions_list = positions_list[:min_length]
        
        # 获取当前时间（秒和纳秒）
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)
        
        # 构建JointState消息
        msg = {
            "header": {
                "stamp": {
                    "sec": sec,
                    "nanosec": nanosec
                },
                "frame_id": frame_id
            },
            "name": joint_names,
            "position": positions_list,
            "velocity": [0.0] * len(positions_list),  # 默认为0
            "effort": [0.0] * len(positions_list)     # 默认为0
        }
        
        # 发布消息
        return self.publish(topic, "sensor_msgs/JointState", msg)
    
    def close(self):
        """关闭WebSocket连接"""
        with self._lock:
            if self.ws:
                try:
                    self.ws.close()
                    print("ROSBridge WebSocket连接已关闭")
                except:
                    pass
            self.connected = False

# 简化使用的全局实例
_rosbridge_client = None

def init_rosbridge(host='127.0.0.1', port=9090, use_ssl=False):
    """初始化全局ROSBridge客户端"""
    global _rosbridge_client
    if _rosbridge_client is None:
        _rosbridge_client = ROSBridgeClient(host, port, use_ssl)
    return _rosbridge_client

def publish_joint_state(topic, joint_positions, joint_names, frame_id="piper_single"):
    """发布关节状态（使用全局客户端）"""
    global _rosbridge_client
    if _rosbridge_client is None:
        raise RuntimeError("请先调用init_rosbridge初始化ROSBridge客户端")
    return _rosbridge_client.publish_joint_state(topic, joint_positions, joint_names, frame_id)

def close_rosbridge():
    """关闭全局ROSBridge客户端"""
    global _rosbridge_client
    if _rosbridge_client is not None:
        _rosbridge_client.close()
        _rosbridge_client = None

# 如果直接运行此模块，执行简单测试
if __name__ == "__main__":
    import argparse
    import time
    
    parser = argparse.ArgumentParser(description="ROSBridge WebSocket客户端测试")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="ROSBridge主机")
    parser.add_argument("--port", type=int, default=9090, help="ROSBridge端口")
    parser.add_argument("--topic", type=str, default="/joint_custom_state", help="要发布的话题")
    parser.add_argument("--ssl", action="store_true", default=False, help="使用SSL连接")
    parser.add_argument("--frame-id", type=str, default="piper_single", help="Frame ID")
    
    args = parser.parse_args()
    
    # 初始化ROSBridge客户端
    client = init_rosbridge(args.host, args.port, args.ssl)
    
    try:
        # 发送10条测试消息
        for i in range(10):
            # 创建测试关节位置
            test_positions = [0.1 * i, 0.2 * i, -0.2 * i, 0.3 * i, -0.2 * i, 0.5 * i, 0.01 * i]
            test_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
            
            # 发布关节状态
            publish_joint_state(args.topic, test_positions, test_names, args.frame_id)
            
            print(f"已发送测试消息 {i+1}/10: {test_positions}")
            time.sleep(1.0)
        
        print("测试完成")
    
    except KeyboardInterrupt:
        print("测试被中断")
    finally:
        # 关闭连接
        close_rosbridge()