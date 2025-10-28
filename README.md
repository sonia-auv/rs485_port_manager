# rs485_port_manager

*description here*

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
| Topic                 | `/provider_rs485/mission_status` | Published       | `sonia_common_ros2/msg/KillStatus`      | template                           |
| Topic                 | `/provider_rs485/kill_status`    | Published       | `sonia_common_ros2/msg/MissionStatus`   | template                           |

### Power

| Type          | Name                                    | Direction       | Message/Service Type                         | Description                       |
| ------------- | --------------------------------------- | --------------- | -------------------------------------------- | --------------------------------- |
| Topic         | `/provider_power/battery_voltages`      | Published       | `sonia_common_ros2/msg/BatteryPowerMessages` | template                          |
| Topic         | `/provider_power/battery_temperatures`  | Published       | `sonia_common_ros2/msg/BatteryPowerMessages` | template                          |
| Topic         | `/provider_power/battery_currents`      | Published       | `sonia_common_ros2/msg/BatteryPowerMessages` | template                          |
| Topic         | `/provider_power/motor_voltages`        | Published       | `sonia_common_ros2/msg/MotorPowerMessages`   | template                          |
| Topic         | `/provider_power/motor_temperatures`    | Published       | `sonia_common_ros2/msg/MotorPowerMessages`   | template                          |
| Topic         | `/provider_power/motor_currents`        | Published       | `sonia_common_ros2/msg/MotorPowerMessages`   | template                          |
| Topic         | `/provider_power/motor_feedback`        | Published       | `sonia_common_ros2/msg/MotorFeedback`        | template                          |


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