# Unitree Feedback Reader

Uses Unitree SDK to send commands, print feedback, and publish JointState.

## Frame Format (16 bytes)

- Header: `0xFD 0xEE`
- Mode byte:
  - bits 0-3: motor ID
  - bits 4-6: status
- Payload:
  - `tau_fbk` (int16)
  - `omega_fbk` (int16)
  - `theta_fbk` (int32)
  - `temp` (int8)
  - `merror + force` (uint16)
- CRC16-CCITT over bytes 0-13 (poly 0x1021, init 0xFFFF)

Conversion:
- $\tau = \tau_{fbk} / 256$
- $\omega = (\omega_{fbk} / 256) * 2\pi$
- $\theta = (\theta_{fbk} / 32768) * 2\pi$

## Build

```bash
cd /home/xzx/unitree/unitree_actuator_sdk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select unitree_feedback_reader
```

## Run

```bash
source /home/xzx/unitree/unitree_actuator_sdk/ros2_ws/install/setup.bash
ros2 run unitree_feedback_reader feedback_reader
```

## Topics

- Publishes `sensor_msgs/JointState` on `/joint_states` (configurable)

## Parameters

- `ports`: serial device list, default `[/dev/ttyUSB0, /dev/ttyUSB1]`
- `bus0_ids`: motor IDs on `/dev/ttyUSB0`, default `[1..6]`
- `bus1_ids`: motor IDs on `/dev/ttyUSB1`, default `[7..12]`
- `baudrate`: default `4000000`
- `print_rate_hz`: default `50`
- `joint_state_topic`: default `/joint_states`
- `joint_names`: list of 12 joint names
- `show_extra`: print tau/temp/merror/force if `true`
- `send_rate_hz`: command send loop rate, default `200`
- `enable_kp`, `enable_kd`, `enable_q`: enable-phase gains/position
- `enable_duration_sec`: seconds to keep enable-phase before zero command

## Notes

- Uses `unitree_actuator_sdk` Python bindings (`lib/unitree_actuator_sdk*.so`).
- The node sends enable commands first, then sends zero command to trigger feedback.
