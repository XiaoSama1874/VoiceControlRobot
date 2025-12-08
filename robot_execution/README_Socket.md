# Socket 服务器配置和使用指南

本文档说明如何在树莓派上配置和启动 Socket 服务器，以便 Mac 客户端通过 TCP Socket 与 ROS2 系统通信。

## 目录

1. [系统要求](#系统要求)
2. [安装和配置](#安装和配置)
3. [启动 Socket 服务器](#启动-socket-服务器)
4. [启动完整系统](#启动完整系统)
5. [验证连接](#验证连接)
6. [故障排查](#故障排查)

## 系统要求

### ROS2 版本与系统兼容性

| ROS2 版本 | 支持的系统 | Python 版本 | 推荐用途 |
|-----------|-----------|-------------|----------|
| **Humble** | Ubuntu 22.04 (Jammy), Debian Bookworm | Python 3.10 | **树莓派推荐** |
| **Jazzy** | Ubuntu 24.04 (Noble) | Python 3.12 | 最新系统 |
| **Iron** | Ubuntu 22.04 (Jammy) | Python 3.10 | 中等系统 |

**重要提示**：
- **Debian Bookworm** 推荐使用 **ROS2 Humble**
- Socket 服务器需要 ROS2 环境来发布控制命令
- 需要 `xarmrob` 包（用于平滑插值）
- 需要 `xarmrob_interfaces` 消息类型

## 安装和配置

### 1. 检查 ROS2 环境

```bash
# 检查已安装的 ROS2 版本
ls /opt/ros/

# 检查当前 ROS2 环境
echo $ROS_DISTRO
```

### 2. 确保 xarmrob 包可用

Socket 服务器需要以下 ROS2 包：
- `xarmrob` - 机器人控制和平滑插值
- `xarmrob_interfaces` - 消息类型定义

如果尚未安装，请参考项目文档安装这些包。

### 3. 配置 Socket 端口

默认端口是 `5005`。可以在启动脚本中修改：

```bash
./start_socket_server.sh [HOST] [PORT]
# 例如：
./start_socket_server.sh 0.0.0.0 5005
```

## 启动 Socket 服务器

### 方法 1: 使用启动脚本（推荐）

```bash
# 导航到 robot_execution 目录
cd /path/to/VoiceControlRobot/robot_execution

# 使脚本可执行（首次运行）
chmod +x start_socket_server.sh

# 使用默认配置启动（监听 0.0.0.0:5005）
./start_socket_server.sh

# 或指定主机和端口
./start_socket_server.sh 0.0.0.0 5005
```

### 方法 2: 手动启动

```bash
# 1. Source ROS2 环境
source /opt/ros/humble/setup.bash  # 或您的 ROS2 版本

# 2. 启动 xarm_automatic（如果尚未运行）
ros2 launch xarmrob xarm_automatic.launch.py

# 3. 在另一个终端启动 socket 服务器
cd /path/to/VoiceControlRobot/robot_execution
python3 socket_server.py 0.0.0.0 5005
```

## 启动完整系统

Socket 服务器需要以下 ROS2 节点运行：

1. **xarm_automatic** - 机器人控制节点
   - `command_xarm` - 关节控制
   - `xarm_kinematics` - 运动学计算

2. **socket_server** - Socket 命令接收服务器

### 启动顺序

```bash
# 终端 1: 启动机器人控制节点
source /opt/ros/humble/setup.bash
ros2 launch xarmrob xarm_automatic.launch.py

# 终端 2: 启动 Socket 服务器
cd /path/to/VoiceControlRobot/robot_execution
source /opt/ros/humble/setup.bash
./start_socket_server.sh
```

启动脚本会自动检查并启动 `xarm_automatic`（如果尚未运行）。

## 验证连接

### 1. 检查节点是否运行

```bash
ros2 node list
# 应该看到:
# socket_robot_server
# command_xarm
# xarm_kinematics
```

### 2. 检查话题

```bash
ros2 topic list
# 应该看到:
# /endpoint_desired
# /gripper_command
```

### 3. 监控命令执行

```bash
# 监控端点命令
ros2 topic echo /endpoint_desired

# 监控抓取命令
ros2 topic echo /gripper_command
```

### 4. 测试 Socket 连接

在 Mac 上运行：

```bash
# 测试连接
python test_rosbridge_connection.py  # 如果已更新为 socket 测试

# 或使用 netcat 测试
nc -zv <树莓派IP> 5005
```

### 5. 测试机器人命令

在 Mac 上运行：

```bash
python test_robot_commands.py
```

## 配置参数

### Socket 服务器参数

- **host**: 绑定地址（默认: `0.0.0.0`，监听所有接口）
- **port**: Socket 端口（默认: `5005`）

### ROS2 节点参数

Socket 服务器节点支持以下参数：

- **command_frequency**: 命令频率（默认: `5` Hz）
- **endpoint_speed**: 端点移动速度（默认: `0.05` m/s）

可以通过 launch 文件或命令行参数设置：

```bash
ros2 run <package> socket_server --ros-args \
    -p command_frequency:=10 \
    -p endpoint_speed:=0.1
```

### 环境变量

Mac 客户端可以通过环境变量配置连接：

```bash
export ME578_RPI_IP_ADDR=192.168.1.76  # 树莓派 IP 地址
```

## 故障排查

### 问题 1: Socket 服务器无法启动

**症状**: 端口被占用或权限错误

**解决方案**:
1. 检查端口是否被占用:
   ```bash
   sudo netstat -tulpn | grep 5005
   ```
2. 检查防火墙设置:
   ```bash
   sudo ufw status
   sudo ufw allow 5005/tcp
   ```
3. 尝试使用不同的端口:
   ```bash
   ./start_socket_server.sh 0.0.0.0 5006
   ```

### 问题 2: 无法从 Mac 连接

**症状**: Mac 客户端连接超时

**解决方案**:
1. 检查网络连通性:
   ```bash
   ping <树莓派IP>
   ```
2. 检查 Socket 服务器是否监听正确地址:
   ```bash
   sudo netstat -tulpn | grep 5005
   # 应该看到: 0.0.0.0:5005 或 <树莓派IP>:5005
   ```
3. 检查防火墙设置（见问题 1）
4. 验证树莓派 IP 地址:
   ```bash
   hostname -I
   ```

### 问题 3: 命令执行但机器人不移动

**症状**: Socket 服务器收到命令但机器人不响应

**解决方案**:
1. 检查 `xarm_automatic` 节点是否运行:
   ```bash
   ros2 node list | grep -E "command_xarm|xarm_kinematics"
   ```
2. 检查话题是否有消息:
   ```bash
   ros2 topic echo /endpoint_desired
   ros2 topic echo /gripper_command
   ```
3. 检查机器人硬件连接
4. 查看 Socket 服务器日志输出

### 问题 4: JSON 解析错误

**症状**: Socket 服务器报告 "Invalid JSON format"

**解决方案**:
1. 检查 Mac 客户端发送的 JSON 格式
2. 验证 JSON 编码（应为 UTF-8）
3. 检查消息大小（不应超过 1024 字节）

### 问题 5: ROS2 节点未找到

**症状**: "No module named 'rclpy'" 或类似错误

**解决方案**:
1. 确保已 source ROS2 环境:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. 检查 ROS2 安装:
   ```bash
   ros2 --version
   ```
3. 确保使用正确的 Python 版本（ROS2 Humble 需要 Python 3.10）

## 日志和调试

### 查看 Socket 服务器日志

Socket 服务器会输出详细的日志信息，包括：
- 连接信息
- 接收到的命令
- 执行结果
- 错误信息

### 启用 ROS2 日志

```bash
# 设置日志级别
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 运行服务器
./start_socket_server.sh
```

## 安全注意事项

1. **防火墙**: 确保只允许可信网络访问 Socket 端口
2. **网络**: Socket 通信未加密，应在可信网络中使用
3. **访问控制**: 考虑添加认证机制（当前版本未实现）

## 性能优化

1. **网络延迟**: 使用快速、可靠的 Wi-Fi 或以太网连接
2. **命令频率**: 根据网络条件调整 `command_frequency` 参数
3. **并发控制**: Socket 服务器使用锁机制防止同时执行多个命令

## 相关文件

- `socket_server.py`: Socket 服务器主程序
- `start_socket_server.sh`: 启动脚本
- `robot_command_receiver.py`: ROSBridge 版本的命令接收器（参考实现）

## 通信协议

### 命令格式（JSON）

```json
{
  "action": "move",
  "x": 0.2,
  "y": 0.0,
  "z": 0.25,
  "action_description": "Move to position"
}
```

或

```json
{
  "action": "grasp",
  "grasp": true,
  "action_description": "Close gripper"
}
```

### 响应格式（JSON）

成功响应：
```json
{
  "status": "success",
  "message": "Robot arm moved to position (0.2, 0.0, 0.25)",
  "position": {"x": 0.2, "y": 0.0, "z": 0.25}
}
```

错误响应：
```json
{
  "status": "error",
  "message": "Missing coordinates (x, y, z required)"
}
```

## 参考资源

- [ROS2 官方文档](https://docs.ros.org/)
- [Python Socket 编程](https://docs.python.org/3/library/socket.html)

