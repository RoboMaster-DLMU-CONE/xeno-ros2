# Xeno Control Guard

这个包提供了使用OneMotor的MotorGuard类来保护can0和can1接口电机的ROS2节点。

## 功能

- 使用OneMotor的MotorGuard类监控CAN总线通信
- 在通信丢失时自动发送安全帧（全零数据）来停止电机
- 支持仿真模式（当XENO_CONTROL_SIMULATE=ON时禁用硬件保护）
- 提供定期状态日志记录
- 节点崩溃时自动重启

## 使用方法

### 独立启动保护节点

```bash
ros2 launch xeno_control_guard xeno_control_guard.launch.py
```

### 与主控制系统一起启动

```bash
ros2 launch xeno_control_guard xeno_control_guard_with_control.launch.py start_control:=true
```

### 仿真模式构建

在仿真模式下构建时，保护功能会被禁用：

```bash
colcon build --cmake-args -DXENO_CONTROL_SIMULATE=ON
```

## 配置

节点会保护以下CAN接口：
- can0 - 基础电机驱动器
- can1 - 机械臂电机驱动器

当检测到通信丢失时，会向每个接口发送包含全零数据的安全帧，使电机停止运动。

## 依赖

- OneMotor库（自动从GitHub获取）
- rclcpp
- 需要硬件CAN接口（非仿真模式）