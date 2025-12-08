#!/usr/bin/env python3
"""
测试 ROSBridge 连接

此脚本用于测试 Mac 侧到树莓派的 ROSBridge 连接。
运行前确保：
1. 树莓派上 ROSBridge 服务器已启动
2. config.py 中 ROSBridge_HOST 和 ROSBridge_PORT 配置正确
3. ROBOT_DEBUG_MODE = False
"""

import sys
import config
from executor_module.rosbridge_client import get_rosbridge_client


def test_connection():
    """测试 ROSBridge 连接"""
    print("=" * 60)
    print("=== ROSBridge 连接测试 ===")
    print("=" * 60)
    print(f"目标地址: {config.ROSBridge_HOST}:{config.ROSBridge_PORT}")
    print(f"通信模式: {config.ROBOT_COMMUNICATION_MODE}")
    print(f"调试模式: {config.ROBOT_DEBUG_MODE}")
    print("-" * 60)
    
    # 检查配置
    if config.ROBOT_COMMUNICATION_MODE != "rosbridge":
        print("⚠ 警告: ROBOT_COMMUNICATION_MODE 不是 'rosbridge'")
        print(f"   当前值: {config.ROBOT_COMMUNICATION_MODE}")
        print("   请在 config.py 中设置 ROBOT_COMMUNICATION_MODE = 'rosbridge'")
        return False
    
    if config.ROBOT_DEBUG_MODE:
        print("⚠ 警告: ROBOT_DEBUG_MODE 为 True，将使用模拟响应")
        print("   建议设置为 False 以进行真实连接测试")
    
    try:
        print("\n正在创建 ROSBridge 客户端...")
        client = get_rosbridge_client()
        
        print("正在尝试连接...")
        if client.connect():
            print("\n✓✓✓ ROSBridge 连接成功！")
            print(f"   已连接到: {config.ROSBridge_HOST}:{config.ROSBridge_PORT}")
            print(f"   命令话题: /robot_commands")
            print(f"   响应话题: /robot_command_response")
            
            # 测试断开连接
            print("\n正在断开连接...")
            client.disconnect()
            print("✓ 连接已断开")
            
            print("\n" + "=" * 60)
            print("✓✓✓ 连接测试通过！")
            print("=" * 60)
            return True
        else:
            print("\n✗✗✗ ROSBridge 连接失败")
            print("\n可能的原因：")
            print("1. ROSBridge 服务器未在树莓派上启动")
            print("2. IP 地址或端口配置错误")
            print("3. 网络连接问题")
            print("4. 防火墙阻止了连接")
            print("\n排查步骤：")
            print(f"   - 检查网络: ping {config.ROSBridge_HOST}")
            print(f"   - 检查端口: nc -zv {config.ROSBridge_HOST} {config.ROSBridge_PORT}")
            print("   - 确认树莓派上 ROSBridge 已启动")
            print("=" * 60)
            return False
            
    except ImportError as e:
        print("\n✗✗✗ 导入错误")
        print(f"   错误: {e}")
        print("\n解决方案：")
        print("   安装 roslibpy: pip install roslibpy")
        return False
    except Exception as e:
        print("\n✗✗✗ 连接测试失败")
        print(f"   错误类型: {type(e).__name__}")
        print(f"   错误信息: {e}")
        import traceback
        print("\n详细错误信息：")
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)

