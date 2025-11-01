# rs485_port_manager

The project opens up a serial connection to allow the onboard computer to communicate, through the **RS485** board, with various system components - inlcuding the IO board, the killSwitch board and the power management module. The data recieved is then parsed and transmitted using **ROS2** communication on the network through their respective topics/services.

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `std_msgs`
* `std_srvs`

### Sonia packages

* `sonia_common_cpp`
* `sonia_common_ros2`

---

## Node

* Name: `rs485_provider`
* Port Name: `/dev/RS485`
* Port type: serial
* Baud Rate: 115200

---

## Registered Topics / Services / Actions

### RS485

| Type                  | Name                             | Direction       | Message/Service Type                    | Description                        |
| --------------------- | -------------------------------- | ----------------| --------------------------------------- | ---------------------------------- |
| Topic                 | `/provider_rs485/mission_status` | Published       | `sonia_common_ros2/msg/KillStatus`      | The status of the mission switch   |
| Topic                 | `/provider_rs485/kill_status`    | Published       | `sonia_common_ros2/msg/MissionStatus`   | The status of the kill switch      |

### Power

| Type          | Name                                    | Direction       | Message/Service Type                         | Description                                |
| ------------- | --------------------------------------- | --------------- | -------------------------------------------- | ------------------------------------------ |
| Topic         | `/provider_power/battery_voltages`      | Published       | `sonia_common_ros2/msg/BatteryPowerMessages` | The measured battery voltages              |
| Topic         | `/provider_power/battery_temperatures`  | Published       | `sonia_common_ros2/msg/BatteryPowerMessages` | The measured battery temperatures          |
| Topic         | `/provider_power/battery_currents`      | Published       | `sonia_common_ros2/msg/BatteryPowerMessages` | The measured battery currents              |
| Topic         | `/provider_power/motor_voltages`        | Published       | `sonia_common_ros2/msg/MotorPowerMessages`   | The measured motor voltages                |
| Topic         | `/provider_power/motor_temperatures`    | Published       | `sonia_common_ros2/msg/MotorPowerMessages`   | The measured motor temperatures            |
| Topic         | `/provider_power/motor_currents`        | Published       | `sonia_common_ros2/msg/MotorPowerMessages`   | The measured motor currents                |
| Topic         | `/provider_power/motor_feedback`        | Published       | `sonia_common_ros2/msg/MotorFeedback`        | Feedback of the motors state               |
| Topic         | `/provider_power/activate_motors`       | Subscribed      | `std_msgs/msg/Bool`                          | Request tp activate/deactivate the motors  |
| Topic         | `/provider_thruster/thruster_pwm`       | Subscribed      | `sonia_common_ros2/msg/MotorPwm`             | Controls the PWM of the motors             |

### Actuator

| Type                  | Name                               | Direction       | Message/Service Type                    | Description                        |
| --------------------- | ---------------------------------- | ----------------| --------------------------------------- | ---------------------------------- |
| Service               | `/provider_actuator/do_action`     | Service Server  | `sonia_common_ros2/srv/ActuatorService` | The service triggers an actuator   |

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select rs485_port_manager --symlink-install
source install/setup.bash
```
---

## Launch Instructions

### Default launch

```bash
ros2 launch rs485_port_manager launch.py
```

---

## Useful ROS 2 Commands

```bash
ros2 node list
ros2 node info /rs485_provider
ros2 topic echo /provider_rs485/kill_status
ros2 param list /rs485_provider
```

---

## References

* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)

---