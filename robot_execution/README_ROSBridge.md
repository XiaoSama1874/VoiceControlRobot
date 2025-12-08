# ROSBridge 服务器配置和使用指南

本文档说明如何在树莓派上配置和启动 ROSBridge 服务器，以便 Mac 客户端通过 WebSocket 与 ROS2 系统通信。

## 目录

1. [安装 ROSBridge Suite](#安装-rosbridge-suite)
2. [启动 ROSBridge 服务器](#启动-rosbridge-服务器)
3. [启动完整系统](#启动完整系统)
4. [验证连接](#验证连接)
5. [故障排查](#故障排查)

## 安装 ROSBridge Suite

### ROS2 Jazzy (推荐)
```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite
```

### ROS2 Humble
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
```

### ROS2 Iron
```bash
sudo apt update
sudo apt install ros-iron-rosbridge-suite
```

### 验证安装
```bash
ros2 pkg list | grep rosbridge
# 应该看到: rosbridge_server
```

## 启动 ROSBridge 服务器

### 方法 1: 使用 Launch 文件（推荐）

```bash
# 使用默认端口 9090
ros2 launch rosbridge_server rosbridge_websocket.launch.py

# 或指定端口
ros2 launch rosbridge_server rosbridge_websocket.launch.py port:=9090

# 或使用自定义 launch 文件
ros2 launch <your_package> rosbridge_server.launch.py port:=9090
```

### 方法 2: 使用启动脚本

```bash
# 使用默认配置（端口 9090）
./start_rosbridge.sh

# 或指定端口和地址
./start_rosbridge.sh 9090 0.0.0.0
```

### 方法 3: 直接启动节点

```bash
ros2 run rosbridge_server rosbridge_websocket \
    --ros-args \
    -p port:=9090 \
    -p address:=0.0.0.0
```

## 启动完整系统

使用 `start_robot_system.sh` 脚本可以一次性启动所有必需的节点：

```bash
# 使用默认配置
./start_robot_system.sh

# 或指定 ROSBridge 端口和地址
./start_robot_system.sh 9090 0.0.0.0
```

这个脚本会按顺序启动：
1. ROSBridge WebSocket 服务器
2. 机器人控制节点（xarm_automatic）
3. 机器人命令接收节点（robot_command_receiver）

## 验证连接

### 1. 检查节点是否运行

```bash
ros2 node list
# 应该看到: rosbridge_websocket
```

### 2. 检查话题

```bash
ros2 topic list
# 应该看到:
# /robot_commands
# /robot_command_response
# /endpoint_desired
# /gripper_command
```

### 3. 监控命令接收

```bash
# 监控接收到的命令
ros2 topic echo /robot_commands

# 监控发送的响应
ros2 topic echo /robot_command_response
```

### 4. 测试 WebSocket 连接

在 Mac 上运行：
```bash
python test_rosbridge_connection.py
```

## 配置参数

### ROSBridge 服务器参数

- **port**: WebSocket 端口（默认: 9090）
- **address**: 绑定地址（默认: 0.0.0.0，监听所有接口）

### 在 Launch 文件中配置

编辑 `rosbridge_server.launch.py`:

```python
parameters=[{
    'port': 9090,
    'address': '0.0.0.0'  # 或特定 IP，如 '192.168.1.100'
}]
```

### 环境变量

可以通过环境变量覆盖配置：
```bash
export ROSBRidge_PORT=9090
export ROSBRidge_ADDRESS=0.0.0.0
```

## 故障排查

### 问题 1: ROSBridge 节点无法启动

**症状**: `ros2 node list` 中看不到 `rosbridge_websocket`

**解决方案**:
1. 检查是否安装了 rosbridge_server:
   ```bash
   ros2 pkg list | grep rosbridge
   ```
2. 检查日志输出:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket.launch.py
   ```
3. 检查端口是否被占用:
   ```bash
   sudo netstat -tulpn | grep 9090
   ```

### 问题 2: 无法从 Mac 连接

**症状**: Mac 客户端连接超时

**解决方案**:
1. 检查网络连通性:
   ```bash
   ping <树莓派IP>
   ```
2. 检查防火墙设置:
   ```bash
   sudo ufw status
   sudo ufw allow 9090/tcp
   ```
3. 检查 ROSBridge 是否监听正确地址:
   ```bash
   sudo netstat -tulpn | grep 9090
   # 应该看到: 0.0.0.0:9090 或 <树莓派IP>:9090
   ```

### 问题 3: 话题未创建

**症状**: `ros2 topic list` 中看不到 `/robot_commands`

**解决方案**:
1. 确认 `robot_command_receiver` 节点已启动
2. 检查节点日志:
   ```bash
   # 查看 robot_command_receiver 输出
   tail -f /tmp/robot_receiver.log
   ```
3. 手动发布测试消息:
   ```bash
   ros2 topic pub /robot_commands std_msgs/String "data: '{\"action\":\"move_home\"}'"
   ```

## 日志文件位置

使用启动脚本时，日志文件保存在：
- ROSBridge: `/tmp/rosbridge.log`
- Robot Control: `/tmp/xarm.log`
- Command Receiver: `/tmp/robot_receiver.log`

查看日志：
```bash
tail -f /tmp/rosbridge.log
```

## 安全注意事项

1. **防火墙**: 确保只允许可信网络访问 ROSBridge 端口
2. **认证**: ROSBridge 默认不提供认证，在生产环境中考虑添加
3. **加密**: 考虑使用 WSS (WebSocket Secure) 进行加密通信

## 性能优化

1. **端口复用**: 如果多个客户端，考虑使用不同的端口
2. **消息大小**: 限制 JSON 消息大小以避免性能问题
3. **连接数**: 监控并发连接数

## 相关文件

- `rosbridge_server.launch.py`: ROSBridge 服务器启动文件
- `start_rosbridge.sh`: ROSBridge 启动脚本
- `start_robot_system.sh`: 完整系统启动脚本
- `robot_command_receiver.py`: 命令接收节点

## 参考资源

- [ROSBridge Suite 官方文档](http://wiki.ros.org/rosbridge_suite)
- [ROSBridge Protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)

