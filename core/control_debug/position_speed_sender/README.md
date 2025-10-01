# Position Speed Sender

这是一个ROS2 Humble节点，用于发送正弦波形式的位置控制命令到 `rm_controller_interface::msg::PositionSpeedCommand` 类型的话题。

## 功能

- 生成正弦波位置命令
- 发送固定的速度命令值（用户可配置）
- 可配置的正弦波参数（振幅、频率、偏移、相位）
- 可配置的发布频率

## 参数

- `amplitude` (double, 默认: 1.0): 正弦波振幅
- `frequency` (double, 默认: 1.0): 正弦波频率 (Hz)
- `offset` (double, 默认: 0.0): 位置偏移量
- `phase` (double, 默认: 0.0): 相位 (弧度)
- `fixed_speed` (double, 默认: 0.0): 固定速度值
- `publish_frequency` (double, 默认: 100.0): 发布频率 (Hz)
- `topic_name` (string, 默认: "position_speed_command"): 发布话题名称

## 编译

```bash
# 在工作空间根目录下
colcon build --packages-select position_speed_sender
```

## 运行

### 使用launch文件运行（推荐）

```bash
# 使用默认参数
ros2 launch position_speed_sender position_speed_sender.launch.py

# 自定义参数
ros2 launch position_speed_sender position_speed_sender.launch.py amplitude:=3.0 frequency:=2.0 fixed_speed:=1.5
```

### 直接运行节点

```bash
# 使用默认参数
ros2 run position_speed_sender position_speed_sender_node

# 使用自定义参数
ros2 run position_speed_sender position_speed_sender_node --ros-args -p amplitude:=3.0 -p frequency:=2.0 -p fixed_speed:=1.5
```

## 消息格式

发布的消息类型为 `rm_controller_interface::msg::PositionSpeedCommand`：

```
float32 speed      # 速度命令（位置的导数）
float32 position   # 位置命令（正弦波）
```

## 正弦波公式

位置：`position = amplitude * sin(2π * frequency * t + phase) + offset`
速度：`speed = fixed_speed` (固定值)

其中 `t` 是从节点启动开始的时间（秒）。

## 监控输出

可以使用以下命令监控发布的话题：

```bash
# 查看话题列表
ros2 topic list

# 监控位置速度命令
ros2 topic echo /position_speed_command

# 查看话题信息
ros2 topic info /position_speed_command
```