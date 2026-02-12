import os
import time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

from unitree_motor_msgs.msg import UnitreeCommand


def _append_sdk_path():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdk_root = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))
    lib_path = os.path.join(sdk_root, "lib")
    if lib_path not in os.sys.path:
        os.sys.path.append(lib_path)


_append_sdk_path()

from unitree_actuator_sdk import (  # noqa: E402
    MotorCmd,
    MotorData,
    MotorMode,
    MotorType,
    SerialPort,
    queryMotorMode,
)


class FixedCommander(Node):
    def __init__(self):
        super().__init__("unitree_fixed_commander")
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("q_des", [0.0] * 12)
        self.declare_parameter("dq_des", [0.0] * 12)
        self.declare_parameter("tau_ff", [0.0] * 12)
        self.declare_parameter("enable_kp", 0.5)
        self.declare_parameter("enable_kd", 0.02)
        self.declare_parameter("enable_q", 0.0)
        self.declare_parameter("enable_duration_sec", 1.0)
        self.declare_parameter("start_topic", "unitree_start")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("offset_topic", "/offset_msg")
        self.declare_parameter("active_ids", [9])
        self.declare_parameter("use_serial", True)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("motor_type", "GO_M8010_6")

        self.publisher = self.create_publisher(UnitreeCommand, "unitree_command", 10)
        self.subscriber = self.create_subscription(
            UnitreeCommand,
            "unitree_command",
            self.on_command_received,
            10,
        )
        start_topic = str(self.get_parameter("start_topic").value)
        self.start_subscriber = self.create_subscription(
            Bool,
            start_topic,
            self.on_start_command,
            10,
        )
        state_topic = str(self.get_parameter("state_topic").value)
        self.state_subscriber = self.create_subscription(
            JointState,
            state_topic,
            self.on_state_received,
            10,
        )
        offset_topic = str(self.get_parameter("offset_topic").value)
        self.offset_subscriber = self.create_subscription(
            Float64MultiArray,
            offset_topic,
            self.on_offset_received,
            10,
        )
        self.last_cmd = UnitreeCommand()
        self.last_cmd.q_des = [0.0] * 12
        self.last_cmd.dq_des = [0.0] * 12
        self.last_cmd.tau_ff = [0.0] * 12
        self.last_cmd.kp = [float(self.get_parameter("kp").value)] * 12
        self.last_cmd.kd = [float(self.get_parameter("kd").value)] * 12
        self.last_initialized = False
        self.enable_start_time = time.monotonic()
        self.start_requested = False
        self.state_received = False
        self.offset_values = [0.0] * 12
        self.offset_received = False
        self.active_ids = [int(x) for x in list(self.get_parameter("active_ids").value)]

        self.use_serial = bool(self.get_parameter("use_serial").value)
        self.serial_port_name = str(self.get_parameter("serial_port").value)
        self.motor_type = self._parse_motor_type(str(self.get_parameter("motor_type").value))
        self.serial = None
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.mode = queryMotorMode(self.motor_type, MotorMode.FOC)
        if self.use_serial:
            try:
                self.serial = SerialPort(self.serial_port_name)
            except Exception as exc:
                self.get_logger().error(f"Failed to open {self.serial_port_name}: {exc}")
                self.use_serial = False

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

    def publish_command(self):
        enable_duration = float(self.get_parameter("enable_duration_sec").value)
        elapsed = time.monotonic() - self.enable_start_time

        if not self.last_initialized:
            enable_q = float(self.get_parameter("enable_q").value)
            enable_kp = float(self.get_parameter("enable_kp").value)
            enable_kd = float(self.get_parameter("enable_kd").value)
            self.last_cmd.q_des = [enable_q] * 12
            self.last_cmd.dq_des = [0.0] * 12
            self.last_cmd.tau_ff = [0.0] * 12
            self.last_cmd.kp = [enable_kp] * 12
            self.last_cmd.kd = [enable_kd] * 12
            self.last_initialized = True

        if self.offset_received:
            self.last_cmd.q_des = [
                q + self.offset_values[i] for i, q in enumerate(self.last_cmd.q_des)
            ]

        self._apply_active_ids_mask(self.last_cmd)

        if self.use_serial:
            self._send_serial(self.last_cmd)

        if elapsed < enable_duration:
            self.publisher.publish(self.last_cmd)
            return

        if not self.state_received:
            self.publisher.publish(self.last_cmd)
            return

        if not self.start_requested:
            self.publisher.publish(self.last_cmd)
            return

        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        q_des = list(self.get_parameter("q_des").value)
        dq_des = list(self.get_parameter("dq_des").value)
        tau_ff = list(self.get_parameter("tau_ff").value)

        if len(q_des) != 12 or len(dq_des) != 12 or len(tau_ff) != 12:
            self.get_logger().error("q_des/dq_des/tau_ff must be length 12")
            self.publisher.publish(self.last_cmd)
            return

        if self.offset_received:
            q_des = [q + self.offset_values[i] for i, q in enumerate(q_des)]
        self.last_cmd.q_des = q_des
        self.last_cmd.dq_des = dq_des
        self.last_cmd.tau_ff = tau_ff
        self.last_cmd.kp = [kp] * 12
        self.last_cmd.kd = [kd] * 12
        self._apply_active_ids_mask(self.last_cmd)
        if self.use_serial:
            self._send_serial(self.last_cmd)
        self.publisher.publish(self.last_cmd)

    def on_command_received(self, msg: UnitreeCommand):
        self.last_cmd = msg
        self.last_initialized = True
        lines = ["Received UnitreeCommand:"]
        for i in range(12):
            lines.append(
                f"  motor[{i}] q={msg.q_des[i]:.4f} dq={msg.dq_des[i]:.4f} "
                f"tau={msg.tau_ff[i]:.4f} kp={msg.kp[i]:.2f} kd={msg.kd[i]:.2f}"
            )
        self.get_logger().info("\n".join(lines))

    def on_start_command(self, msg: Bool):
        if msg.data and not self.start_requested:
            self.start_requested = True
            self.get_logger().info("Start command received: switching to control mode")

    def on_state_received(self, msg: JointState):
        if not self.state_received:
            self.state_received = True
            self.get_logger().info("Motor state received, enable phase complete")

    def on_offset_received(self, msg: Float64MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn("offset_msg must be length 12")
            return
        self.offset_values = [float(x) for x in msg.data]
        if not self.offset_received:
            self.get_logger().info("Offset received, applying to q_des")
        self.offset_received = True

    def _apply_active_ids_mask(self, cmd: UnitreeCommand):
        if not self.active_ids:
            return
        active_zero_based = {i - 1 for i in self.active_ids}
        for i in range(12):
            if i not in active_zero_based:
                cmd.q_des[i] = 0.0
                cmd.dq_des[i] = 0.0
                cmd.tau_ff[i] = 0.0
                cmd.kp[i] = 0.0
                cmd.kd[i] = 0.0

    def _send_serial(self, cmd_msg: UnitreeCommand):
        if not self.serial:
            return
        motor_id = self.active_ids[0] if self.active_ids else 1
        index = motor_id - 1
        if index < 0 or index >= 12:
            self.get_logger().warn(f"active_ids out of range: {motor_id}")
            return
        self.data.motorType = self.motor_type
        self.cmd.motorType = self.motor_type
        self.cmd.mode = self.mode
        self.cmd.id = int(motor_id)
        self.cmd.q = float(cmd_msg.q_des[index])
        self.cmd.dq = float(cmd_msg.dq_des[index])
        self.cmd.kp = float(cmd_msg.kp[index])
        self.cmd.kd = float(cmd_msg.kd[index])
        self.cmd.tau = float(cmd_msg.tau_ff[index])
        try:
            self.serial.sendRecv(self.cmd, self.data)
        except Exception as exc:
            self.get_logger().warn(f"Serial sendRecv failed: {exc}")

    def _parse_motor_type(self, name: str) -> MotorType:
        try:
            return MotorType[name]
        except Exception:
            self.get_logger().warn(f"Unknown motor type '{name}', fallback GO_M8010_6")
            return MotorType.GO_M8010_6


def main():
    rclpy.init()
    node = FixedCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
