#!/usr/bin/env python3
"""
测试机器人命令执行

此脚本用于测试通过 ROSBridge 发送机器人命令并接收响应。
运行前确保：
1. ROSBridge 连接测试通过（运行 test_rosbridge_connection.py）
2. 树莓派上 robot_command_receiver 节点已启动
3. config.py 中配置正确
4. ROBOT_DEBUG_MODE = False
"""

import sys
import time
import config
from executor_module.robot_functions import move_home, move, grasp


def print_separator(title=""):
    """打印分隔线"""
    if title:
        print("\n" + "=" * 60)
        print(f"=== {title} ===")
        print("=" * 60)
    else:
        print("-" * 60)


def test_move_home():
    """测试 move_home 命令"""
    print_separator("测试 move_home")
    print("执行: move_home()")
    print(f"目标位置: {config.ROBOT_HOME_POSITION}")
    
    try:
        result = move_home()
        print(f"\n返回结果:")
        print(f"  状态: {result.get('status')}")
        print(f"  消息: {result.get('message')}")
        print(f"  位置: {result.get('position')}")
        
        if result["status"] == "success":
            print("\n✓ move_home 测试通过")
            return True
        else:
            print("\n✗ move_home 测试失败")
            return False
    except Exception as e:
        print(f"\n✗ move_home 执行出错: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_move():
    """测试 move 命令"""
    print_separator("测试 move")
    test_position = {"x": 0.15, "y": 0.05, "z": 0.20}
    print(f"执行: move(x={test_position['x']}, y={test_position['y']}, z={test_position['z']})")
    
    try:
        result = move(x=test_position["x"], y=test_position["y"], z=test_position["z"])
        print(f"\n返回结果:")
        print(f"  状态: {result.get('status')}")
        print(f"  消息: {result.get('message')}")
        print(f"  位置: {result.get('position')}")
        
        if result["status"] == "success":
            print("\n✓ move 测试通过")
            return True
        else:
            print("\n✗ move 测试失败")
            return False
    except Exception as e:
        print(f"\n✗ move 执行出错: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_grasp():
    """测试 grasp 命令"""
    results = []
    
    # 测试关闭夹爪
    print_separator("测试 grasp (关闭)")
    print("执行: grasp(True)")
    
    try:
        result = grasp(True)
        print(f"\n返回结果:")
        print(f"  状态: {result.get('status')}")
        print(f"  消息: {result.get('message')}")
        print(f"  夹爪状态: {result.get('grasp_state')}")
        
        if result["status"] == "success":
            print("\n✓ grasp (关闭) 测试通过")
            results.append(True)
        else:
            print("\n✗ grasp (关闭) 测试失败")
            results.append(False)
    except Exception as e:
        print(f"\n✗ grasp (关闭) 执行出错: {e}")
        import traceback
        traceback.print_exc()
        results.append(False)
    
    # 等待一下
    time.sleep(0.5)
    
    # 测试打开夹爪
    print_separator("测试 grasp (打开)")
    print("执行: grasp(False)")
    
    try:
        result = grasp(False)
        print(f"\n返回结果:")
        print(f"  状态: {result.get('status')}")
        print(f"  消息: {result.get('message')}")
        print(f"  夹爪状态: {result.get('grasp_state')}")
        
        if result["status"] == "success":
            print("\n✓ grasp (打开) 测试通过")
            results.append(True)
        else:
            print("\n✗ grasp (打开) 测试失败")
            results.append(False)
    except Exception as e:
        print(f"\n✗ grasp (打开) 执行出错: {e}")
        import traceback
        traceback.print_exc()
        results.append(False)
    
    return all(results)


def test_sequence():
    """测试命令序列"""
    print_separator("测试命令序列")
    print("将执行以下命令序列：")
    print("1. move_home() - 回到初始位置")
    print("2. move(x=0.15, y=0.05, z=0.20) - 移动到目标位置")
    print("3. grasp(True) - 关闭夹爪")
    print("4. move(x=0.15, y=0.05, z=0.30) - 抬起")
    print("5. grasp(False) - 打开夹爪")
    print("6. move_home() - 回到初始位置")
    
    input("\n按 Enter 键开始执行序列（或 Ctrl+C 取消）...")
    
    try:
        print("\n[1/6] 执行 move_home()...")
        result = move_home()
        if result["status"] != "success":
            print(f"✗ 步骤 1 失败: {result.get('message')}")
            return False
        print("✓ 步骤 1 完成")
        time.sleep(1)
        
        print("\n[2/6] 执行 move(x=0.15, y=0.05, z=0.20)...")
        result = move(x=0.15, y=0.05, z=0.20)
        if result["status"] != "success":
            print(f"✗ 步骤 2 失败: {result.get('message')}")
            return False
        print("✓ 步骤 2 完成")
        time.sleep(1)
        
        print("\n[3/6] 执行 grasp(True)...")
        result = grasp(True)
        if result["status"] != "success":
            print(f"✗ 步骤 3 失败: {result.get('message')}")
            return False
        print("✓ 步骤 3 完成")
        time.sleep(1)
        
        print("\n[4/6] 执行 move(x=0.15, y=0.05, z=0.30)...")
        result = move(x=0.15, y=0.05, z=0.30)
        if result["status"] != "success":
            print(f"✗ 步骤 4 失败: {result.get('message')}")
            return False
        print("✓ 步骤 4 完成")
        time.sleep(1)
        
        print("\n[5/6] 执行 grasp(False)...")
        result = grasp(False)
        if result["status"] != "success":
            print(f"✗ 步骤 5 失败: {result.get('message')}")
            return False
        print("✓ 步骤 5 完成")
        time.sleep(1)
        
        print("\n[6/6] 执行 move_home()...")
        result = move_home()
        if result["status"] != "success":
            print(f"✗ 步骤 6 失败: {result.get('message')}")
            return False
        print("✓ 步骤 6 完成")
        
        print("\n✓✓✓ 命令序列测试完成！")
        return True
        
    except KeyboardInterrupt:
        print("\n\n用户取消执行")
        return False
    except Exception as e:
        print(f"\n✗ 命令序列执行出错: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主测试函数"""
    print("=" * 60)
    print("=== 机器人命令执行测试 ===")
    print("=" * 60)
    print(f"通信模式: {config.ROBOT_COMMUNICATION_MODE}")
    print(f"调试模式: {config.ROBOT_DEBUG_MODE}")
    print(f"ROSBridge 地址: {config.ROSBridge_HOST}:{config.ROSBridge_PORT}")
    print("=" * 60)
    
    # 检查配置
    if config.ROBOT_COMMUNICATION_MODE != "rosbridge":
        print("\n✗ 错误: ROBOT_COMMUNICATION_MODE 不是 'rosbridge'")
        print(f"   当前值: {config.ROBOT_COMMUNICATION_MODE}")
        print("   请在 config.py 中设置 ROBOT_COMMUNICATION_MODE = 'rosbridge'")
        return False
    
    if config.ROBOT_DEBUG_MODE:
        print("\n⚠ 警告: ROBOT_DEBUG_MODE 为 True")
        print("   将使用模拟响应，不会进行真实通信")
        print("   建议设置为 False 以进行真实测试")
        response = input("\n是否继续？(y/n): ")
        if response.lower() != 'y':
            return False
    
    # 运行测试
    results = []
    
    # 基础功能测试
    print("\n" + "=" * 60)
    print("开始基础功能测试...")
    print("=" * 60)
    
    results.append(("move_home", test_move_home()))
    time.sleep(1)
    
    results.append(("move", test_move()))
    time.sleep(1)
    
    results.append(("grasp", test_grasp()))
    time.sleep(1)
    
    # 汇总结果
    print("\n" + "=" * 60)
    print("=== 测试结果汇总 ===")
    print("=" * 60)
    for test_name, success in results:
        status = "✓ 通过" if success else "✗ 失败"
        print(f"{test_name:20s}: {status}")
    
    all_passed = all(result[1] for result in results)
    
    if all_passed:
        print("\n✓✓✓ 所有基础测试通过！")
        
        # 询问是否运行序列测试
        print("\n" + "=" * 60)
        response = input("是否运行命令序列测试？(y/n): ")
        if response.lower() == 'y':
            sequence_result = test_sequence()
            if sequence_result:
                print("\n✓✓✓ 所有测试通过！")
                return True
            else:
                print("\n✗ 命令序列测试失败")
                return False
        else:
            return True
    else:
        print("\n✗ 部分测试失败，请检查配置和连接")
        return False


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ 测试程序出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

