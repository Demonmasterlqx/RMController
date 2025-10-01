# Position Speed Sender

这是一个ROS2 Humble节点，用于发送正弦波形式的位置控制命令到 `rm_controller_interface::msg::PositionSpeedCommand` 类型的话题。

## 功能

支持三种工作模式：

1. **模式1 - 位置正弦波 + 速度固定**: 生成正弦波位置命令，发送固定的速度命令值
2. **模式2 - 速度正弦波 + 位置固定**: 生成正弦波速度命令，发送固定的位置命令值
3. **模式3 - 位置正弦波 + 速度求导**: 生成正弦波位置命令，速度为位置的导数

- 可配置的正弦波参数（振幅、频率、偏移、相位）
- 可配置的发布频率

## 参数

- `work_mode` (string, 默认: "position_sine_speed_fixed"): 工作模式
  - "position_sine_speed_fixed" 或 "1": 模式1 - 位置正弦波 + 速度固定
  - "speed_sine_position_fixed" 或 "2": 模式2 - 速度正弦波 + 位置固定
  - "position_sine_speed_derived" 或 "3": 模式3 - 位置正弦波 + 速度求导
- `amplitude` (double, 默认: 1.0): 正弦波振幅
- `frequency` (double, 默认: 1.0): 正弦波频率 (Hz)
- `offset` (double, 默认: 0.0): 偏移量
- `phase` (double, 默认: 0.0): 相位 (弧度)
- `fixed_speed` (double, 默认: 0.0): 固定速度值（模式1使用）
- `fixed_position` (double, 默认: 0.0): 固定位置值（模式2使用）
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
# 使用默认参数（模式1）
ros2 launch position_speed_sender position_speed_sender.launch.py

# 运行模式2：速度正弦波 + 位置固定
ros2 launch position_speed_sender position_speed_sender.launch.py work_mode:=speed_sine_position_fixed fixed_position:=1.57

# 运行模式3：位置正弦波 + 速度求导
ros2 launch position_speed_sender position_speed_sender.launch.py work_mode:=position_sine_speed_derived

# 使用数字指定模式
ros2 launch position_speed_sender position_speed_sender.launch.py work_mode:=2
```

### 直接运行节点

```bash
# 模式1：位置正弦波 + 速度固定
ros2 run position_speed_sender position_speed_sender_node --ros-args -p work_mode:=1 -p fixed_speed:=2.0

# 模式2：速度正弦波 + 位置固定
ros2 run position_speed_sender position_speed_sender_node --ros-args -p work_mode:=2 -p fixed_position:=1.57

# 模式3：位置正弦波 + 速度求导
ros2 run position_speed_sender position_speed_sender_node --ros-args -p work_mode:=3

# 使用字符串指定模式
ros2 run position_speed_sender position_speed_sender_node --ros-args -p work_mode:=speed_sine_position_fixed
```

## 消息格式

发布的消息类型为 `rm_controller_interface::msg::PositionSpeedCommand`：

```
float32 speed      # 速度命令
float32 position   # 位置命令
```

## 应用场景

- **模式1**: 适用于测试位置控制器，同时给定固定速度前馈
- **模式2**: 适用于测试速度控制器，保持位置不变
- **模式3**: 适用于测试完整的位置轨迹跟踪，速度与位置完全同步

## 数学公式

**模式1 - 位置正弦波 + 速度固定**
- 位置：`position = amplitude * sin(2π * frequency * t + phase) + offset`
- 速度：`speed = fixed_speed` (固定值)

**模式2 - 速度正弦波 + 位置固定**
- 位置：`position = fixed_position` (固定值)
- 速度：`speed = amplitude * sin(2π * frequency * t + phase) + offset`

**模式3 - 位置正弦波 + 速度求导**
- 位置：`position = amplitude * sin(2π * frequency * t + phase) + offset`
- 速度：`speed = amplitude * frequency * 2π * cos(2π * frequency * t + phase)`

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

# 查看消息频率
ros2 topic hz /position_speed_command

# 绘制数据图表（需要安装rqt_plot）
rqt_plot /position_speed_command/position /position_speed_command/speed
```

## 常用参数组合示例

```bash
# 低频大幅度位置正弦波 + 固定速度
ros2 launch position_speed_sender position_speed_sender.launch.py \
  work_mode:=position_sine_speed_fixed \
  amplitude:=5.0 frequency:=0.2 fixed_speed:=1.0

# 高频小幅度速度正弦波
ros2 launch position_speed_sender position_speed_sender.launch.py \
  work_mode:=speed_sine_position_fixed \
  amplitude:=2.0 frequency:=2.0 fixed_position:=0.0

# 标准正弦轨迹（位置和速度同步）
ros2 launch position_speed_sender position_speed_sender.launch.py \
  work_mode:=position_sine_speed_derived \
  amplitude:=3.14159 frequency:=0.5 offset:=0.0

ros2 launch position_speed_sender position_speed_sender.launch.py \
  work_mode:=position_sine_speed_fixed \
  amplitude:=2.0 \
  frequency:=1.0 \
  offset:=0.0 \
  phase:=0.0 \
  fixed_speed:=5.0 \
  fixed_position:=0.0 \
  publish_frequency:=100.0 \
  topic_name:=position_speed_command

```