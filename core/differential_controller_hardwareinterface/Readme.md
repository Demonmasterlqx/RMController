# differential_controller_hardware_interface

这是一个曲线救国的补丁，由于ros2 humble的chainable没有提供 `export_state_interface` 方法，导致其不能暴露state接口，让末端差速器无法工作。

这个pkg的目的是将differential_controller的state的topic桥接成为 hardwareinterfce 的一部分

参数设置

```xml

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

        <ros2_control name="differential_controller_hardwareinterface_test_robot_control" type="system">
            <hardware>
                <plugin>RM_hardware_interface/DifferentialControllerHardwareInterface</plugin>
            </hardware>

            <joint name="joint">

                <param name="controller_name">differential_controller</param>

            </joint>

        </ros2_control>
</robot>

```

这个 hardwareinterface 将会监听 `controller_name/joint_name/state/velocity` 和 `controller_name/joint_name/state/position` topic，并将其转换为 interface 中的内容