# Differential Command Sender

这是一个ROS2 Humble节点，用于发送正弦波形式的差分控制命令到 `rm_controller_interface::msg::EndDifferentialCommand` 类型的话题。

## 功能

支持六种工作模式：

1. **模式1 - 位置正弦波 + 速度固定**: 生成正弦波位置命令，发送固定的速度命令值
2. **模式2 - 速度正弦波 + 位置固定**: 生成正弦波速度命令，发送固定的位置命令值
3. **模式3 - 位置正弦波 + 速度求导**: 生成正弦波位置命令，速度为位置的导数
4. **模式4 - roll正弦 + pitch固定 + 速度固定**: roll轴正弦波位置，pitch轴固定位置，速度固定
5. **模式5 - roll固定 + pitch正弦 + 速度固定**: roll轴固定位置，pitch轴正弦波位置，速度固定
6. **模式6 - roll和pitch均正弦 + 速度固定**: 两轴都是正弦波位置（有相位差），速度固定

- 可配置的正弦波参数（振幅、频率、偏移、相位）
- 可配置的发布频率
- 支持roll和pitch两个轴的差分控制

## 参数

- `work_mode` (string, 默认: "position_sine_velocity_fixed"): 工作模式
  - "position_sine_velocity_fixed" 或 "1": 模式1 - 位置正弦波 + 速度固定
  - "velocity_sine_position_fixed" 或 "2": 模式2 - 速度正弦波 + 位置固定
  - "position_sine_velocity_derived" 或 "3": 模式3 - 位置正弦波 + 速度求导
  - "roll_sine_pitch_fixed" 或 "4": 模式4 - roll正弦 + pitch固定 + 速度固定
  - "roll_fixed_pitch_sine" 或 "5": 模式5 - roll固定 + pitch正弦 + 速度固定
  - "roll_pitch_both_sine" 或 "6": 模式6 - roll和pitch均正弦 + 速度固定
- `amplitude` (double, 默认: 1.0): 正弦波振幅
- `frequency` (double, 默认: 1.0): 正弦波频率 (Hz)
- `offset` (double, 默认: 0.0): 偏移量
- `phase` (double, 默认: 0.0): 相位 (弧度)
- `fixed_velocity` (double, 默认: 0.0): 固定速度值（模式1使用）
- `fixed_position` (double, 默认: 0.0): 固定位置值（模式2使用）
- `publish_frequency` (double, 默认: 100.0): 发布频率 (Hz)
- `topic_name` (string, 默认: "end_differential_command"): 发布话题名称

## 编译

```bash
# 在工作空间根目录下
colcon build --packages-select differential_command_sender
```

## 运行

### 使用launch文件运行（推荐）

```bash
# 使用默认参数（模式1）
ros2 launch differential_command_sender differential_command_sender.launch.py

# 运行模式2：速度正弦波 + 位置固定
ros2 launch differential_command_sender differential_command_sender.launch.py work_mode:=velocity_sine_position_fixed fixed_position:=1.57

# 运行模式3：位置正弦波 + 速度求导
ros2 launch differential_command_sender differential_command_sender.launch.py work_mode:=position_sine_velocity_derived

# 运行模式4：roll正弦 + pitch固定
ros2 launch differential_command_sender differential_command_sender.launch.py work_mode:=roll_sine_pitch_fixed fixed_position:=1.57 fixed_velocity:=2.0

# 运行模式5：roll固定 + pitch正弦
ros2 launch differential_command_sender differential_command_sender.launch.py work_mode:=roll_fixed_pitch_sine fixed_position:=0.0 fixed_velocity:=1.5

# 运行模式6：roll和pitch均正弦
ros2 launch differential_command_sender differential_command_sender.launch.py work_mode:=roll_pitch_both_sine fixed_velocity:=1.0

# 使用数字指定模式
ros2 launch differential_command_sender differential_command_sender.launch.py work_mode:=4
```

### 直接运行节点

```bash
# 模式1：位置正弦波 + 速度固定
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=1 -p fixed_velocity:=2.0

# 模式2：速度正弦波 + 位置固定
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=2 -p fixed_position:=1.57

# 模式3：位置正弦波 + 速度求导
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=3

# 模式4：roll正弦 + pitch固定
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=4 -p fixed_position:=1.57 -p fixed_velocity:=2.0

# 模式5：roll固定 + pitch正弦
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=5 -p fixed_position:=0.0 -p fixed_velocity:=1.5

# 模式6：roll和pitch均正弦
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=6 -p fixed_velocity:=1.0

# 使用字符串指定模式
ros2 run differential_command_sender differential_command_sender_node --ros-args -p work_mode:=roll_sine_pitch_fixed
```

## 消息格式

发布的消息类型为 `rm_controller_interface::msg::EndDifferentialCommand`：

```
float32 roll_position    # roll轴位置命令
float32 roll_velocity    # roll轴速度命令
float32 pitch_position   # pitch轴位置命令
float32 pitch_velocity   # pitch轴速度命令
```

## 应用场景

- **模式1**: 适用于测试差分控制器的位置控制，同时给定固定速度前馈
- **模式2**: 适用于测试差分控制器的速度控制，保持位置不变
- **模式3**: 适用于测试完整的差分轨迹跟踪，速度与位置完全同步
- **模式4**: 适用于测试单轴（roll）的位置控制，pitch轴保持固定位置
- **模式5**: 适用于测试单轴（pitch）的位置控制，roll轴保持固定位置
- **模式6**: 适用于测试双轴协调位置控制，两轴都运动但有相位差

## 数学公式

**模式1 - 位置正弦波 + 速度固定**
- roll位置：`roll_position = amplitude * sin(2π * frequency * t + phase) + offset`
- roll速度：`roll_velocity = fixed_velocity` (固定值)
- pitch位置：`pitch_position = amplitude * sin(2π * frequency * t + phase + π/4) + offset`
- pitch速度：`pitch_velocity = fixed_velocity` (固定值)

**模式2 - 速度正弦波 + 位置固定**
- roll位置：`roll_position = fixed_position` (固定值)
- roll速度：`roll_velocity = amplitude * sin(2π * frequency * t + phase) + offset`
- pitch位置：`pitch_position = fixed_position` (固定值)
- pitch速度：`pitch_velocity = amplitude * sin(2π * frequency * t + phase + π/4) + offset`

**模式3 - 位置正弦波 + 速度求导**
- roll位置：`roll_position = amplitude * sin(2π * frequency * t + phase) + offset`
- roll速度：`roll_velocity = amplitude * frequency * 2π * cos(2π * frequency * t + phase)`
- pitch位置：`pitch_position = amplitude * sin(2π * frequency * t + phase + π/4) + offset`
- pitch速度：`pitch_velocity = amplitude * frequency * 2π * cos(2π * frequency * t + phase + π/4)`

**模式4 - roll正弦 + pitch固定 + 速度固定**
- roll位置：`roll_position = amplitude * sin(2π * frequency * t + phase) + offset`
- roll速度：`roll_velocity = fixed_velocity` (固定值)
- pitch位置：`pitch_position = fixed_position` (固定值)
- pitch速度：`pitch_velocity = fixed_velocity` (固定值)

**模式5 - roll固定 + pitch正弦 + 速度固定**
- roll位置：`roll_position = fixed_position` (固定值)
- roll速度：`roll_velocity = fixed_velocity` (固定值)
- pitch位置：`pitch_position = amplitude * sin(2π * frequency * t + phase) + offset`
- pitch速度：`pitch_velocity = fixed_velocity` (固定值)

**模式6 - roll和pitch均正弦 + 速度固定**
- roll位置：`roll_position = amplitude * sin(2π * frequency * t + phase) + offset`
- roll速度：`roll_velocity = fixed_velocity` (固定值)
- pitch位置：`pitch_position = amplitude * sin(2π * frequency * t + phase + π/4) + offset`
- pitch速度：`pitch_velocity = fixed_velocity` (固定值)

其中 `t` 是从节点启动开始的时间（秒）。

注意：在模式1、3、6中，pitch轴相比roll轴有π/4的相位差，这样可以产生更丰富的差分运动模式。

## 监控输出

节点会每秒打印一次当前的命令值，便于监控和调试：

```
[INFO] [differential_command_sender]: Time: 1.234s, Roll(pos: 1.234, vel: 0.567), Pitch(pos: 0.890, vel: 0.567)
```

## 常用参数组合示例

```bash
# 低频大幅度位置正弦波 + 固定速度
ros2 launch differential_command_sender differential_command_sender.launch.py \
  work_mode:=position_sine_velocity_fixed \
  amplitude:=5.0 frequency:=0.2 fixed_velocity:=1.0

# 高频小幅度速度正弦波
ros2 launch differential_command_sender differential_command_sender.launch.py \
  work_mode:=velocity_sine_position_fixed \
  amplitude:=2.0 frequency:=2.0 fixed_position:=0.0

# 标准正弦轨迹（位置和速度同步）
ros2 launch differential_command_sender differential_command_sender.launch.py \
  work_mode:=position_sine_velocity_derived \
  amplitude:=3.14159 frequency:=0.5 offset:=0.0

# 测试roll轴单独运动
ros2 launch differential_command_sender differential_command_sender.launch.py \
  work_mode:=roll_sine_pitch_fixed \
  amplitude:=2.0 frequency:=1.0 fixed_position:=1.57 fixed_velocity:=1.5

# 测试pitch轴单独运动
ros2 launch differential_command_sender differential_command_sender.launch.py \
  work_mode:=roll_fixed_pitch_sine \
  amplitude:=1.5 frequency:=0.8 fixed_position:=0.0 fixed_velocity:=2.0

# 双轴协调运动
ros2 launch differential_command_sender differential_command_sender.launch.py \
  work_mode:=roll_pitch_both_sine \
  amplitude:=3.0 frequency:=0.3 offset:=1.57 fixed_velocity:=1.0

# 自定义话题名称和高频发布
ros2 launch differential_command_sender differential_command_sender.launch.py \
  topic_name:=custom_differential_command \
  publish_frequency:=1000.0 \
  work_mode:=roll_pitch_both_sine
```