# differential_controller

这是一个用于末端差速器控制的控制器，给定两个电机的速度位置，暴露差速器控制的两个关节的速度位置

控制器提供标零，差速器角度解算基于差速器标零后两个电机的位置，也就是基于标零后旋转的角度。

输入控制指令的时候需要保证不要让电机转多圈。也就是说，在控制指令期望状态之间和当前状态之间，解算出来的实际电机需要的旋转角度不要让电机套圈。

一般来说，有轨迹规划器的输入都是没问题的

控制器启动的时候，会自动标零一次

控制器会在topic controller_name/joint/XXX 下发布 实际转动电机和差速器控制关节的 position 和 velocity 的 ref 和 state，类型为 std_msgs::msg::Float32

控制器会在topic controller_name/is_zero 下发布标零状态，如果标零失败，则会发布 -1，标零中发布 0, 标零成功发布 1

position/veloctiy 的单位是 rad rad/s

需要给出的参数：


```yaml

differential_controller:
    # 实际控制的电机，需要有 command 接口 velocity && position， state 接口 velocity && position && effort（用于标零，检测是否处于使能状态）
    # 定义左右电机的转轴指向分别为向左和向右
    motor_left: motor_left
    motor_right: motor_right

    # 可选参数，直接指定对应的state和command接口，会覆盖上面提供的接口
    motor_left_state_position: 
    motor_left_state_velocity:
    motor_left_state_effort:
    motor_left_command_position:
    motor_left_command_velocity:
    motor_right_state_position:
    motor_right_state_velocity:
    motor_right_state_effort:
    motor_right_command_position:
    motor_right_command_velocity:

    # 齿轮传动比 电机直接连接的齿轮比上连接两个电机的齿轮的齿轮
    gear_ratio: 0.5
    # 输出的关节的名称，控制器会给出这两个轴的 state && command 接口，都包含 velocity && position 接口
    # 指的是当两个电机分别以相同速度但是方向不同转动时动的轴
    pitch_joint: pitch_joint
    # 指的是当两个电机分别以相同速度方向转动时动的轴
    roll_joint: roll_joint
    # 标零参数
    # 在标零时。控制器将会向电机发出 当前位置 +- zero_delta_position 的值作为 command position 的输入
    zero_delta_position: 0.1
    # 在标零时。控制器将会向电机发出 zero_velocity 的值作为 command velocity 的输入
    zero_velocity: 1.0
    # 两个电机的position的最大值-最小值，用于数圈数
    maximum_rotational_range: 6.28

    # 标零topic，类型为 std_msg::msg::Bool，如果发布一个 true 的消息，控制器会进入标零状态尝试控制电机标零
    zero_topic: zero_topic

    # 强制标零topic，类型为 std_msg::msg::Bool，如果发布一个 true 的消息，控制器会将当前的位置视作零点位置
    force_zero_topic: force_zero_topic

    # 标零状态下的roll的position
    zero_roll_position: 0.0
    # 标零状态下的pitch的position
    zero_pitch_position: 1.5555
    # 标零时间限制
    zero_timeout: 5.0
    # 当检测到电机处于静止状态时发出的速度置零(实现电机定死不动)
    stable_velocity_command: 1.0

```

## 标零过程

控制器会通过 state effort 查看电机是否使能（如果使能了，在控制的过程中应该会有明显的力矩变化）

控制器会根据 zero_delta_position 和 zero_velocity 控制控制电机，值到他们的位置不再有明显变化，然后电机会将这个位置作为基准位置。

## 注意

1. 不要让电机在没有处于置零状态的情况下被机械限位！！