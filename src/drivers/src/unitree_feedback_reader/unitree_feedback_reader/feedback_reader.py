import os
import sys
import threading
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from unitree_motor_msgs.msg import UnitreeCommand


def _append_sdk_path():
    candidates = []
    env_root = os.environ.get("UNITREE_ACTUATOR_SDK_ROOT")
    if env_root:
        candidates.append(env_root)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.abspath(os.path.join(current_dir, "..", "..", "..", "..")))

    colcon_prefix = os.environ.get("COLCON_CURRENT_PREFIX")
    if colcon_prefix:
        candidates.append(os.path.abspath(os.path.join(colcon_prefix, "..", "..")))

    ament_prefix = os.environ.get("AMENT_PREFIX_PATH")
    if ament_prefix:
        first_prefix = ament_prefix.split(os.pathsep)[0]
        candidates.append(os.path.abspath(os.path.join(first_prefix, "..", "..")))

    candidates.append(os.path.abspath(os.path.join(os.getcwd(), "..")))

    for root in candidates:
        lib_path = os.path.join(root, "lib")
        if os.path.isdir(lib_path) and lib_path not in sys.path:
            sys.path.append(lib_path)
            return


_append_sdk_path()

from unitree_actuator_sdk import (  # noqa: E402
    MotorCmd,
    MotorData,
    MotorMode,
    MotorType,
    SerialPort,
    queryGearRatio,
    queryMotorMode,
)


class FeedbackReader(Node):
    def __init__(self):
        super().__init__("unitree_feedback_reader")
        self.declare_parameter("ports", ["/dev/ttyUSB2", "/dev/ttyUSB3"])
        self.declare_parameter("bus0_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("bus1_ids", [7, 8, 9, 10, 11, 12])
        self.declare_parameter("baudrate", 4000000)
        self.declare_parameter("print_rate_hz", 50.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/unitree_command")
        self.declare_parameter("joint_names", [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "joint_8",
            "joint_9",
            "joint_10",
            "joint_11",
            "joint_12",
        ])
        self.declare_parameter("show_extra", False)
        self.declare_parameter("send_rate_hz", 200.0)
        self.declare_parameter("enable_kp", 0.5)
        self.declare_parameter("enable_kd", 0.02)
        self.declare_parameter("enable_q", 0.0)
        self.declare_parameter("enable_duration_sec", 1.0)
        self.declare_parameter("offset_window_sec", 1.0)
        self.declare_parameter("stale_timeout_sec", 0.2)
        self.declare_parameter("command_stale_timeout_sec", 0.5)
        self.declare_parameter("leg_feedback_mask", [0, 0, 0, 1])

        self.baudrate = int(self.get_parameter("baudrate").value)
        self.print_rate = float(self.get_parameter("print_rate_hz").value)
        self.show_extra = bool(self.get_parameter("show_extra").value)
        self.send_rate = float(self.get_parameter("send_rate_hz").value)
        self.enable_kp = float(self.get_parameter("enable_kp").value)
        self.enable_kd = float(self.get_parameter("enable_kd").value)
        self.enable_q = float(self.get_parameter("enable_q").value)
        self.enable_duration = float(self.get_parameter("enable_duration_sec").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.offset_window_sec = float(self.get_parameter("offset_window_sec").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self.command_stale_timeout_sec = float(
            self.get_parameter("command_stale_timeout_sec").value
        )
        self.leg_feedback_mask = self._normalize_leg_mask(
            list(self.get_parameter("leg_feedback_mask").value)
        )

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.offset_pub = self.create_publisher(Float64MultiArray, "offset_msg", 10)
        self.command_sub = self.create_subscription(
            UnitreeCommand,
            self.command_topic,
            self.on_command_received,
            10,
        )

        self.latest: Dict[int, Dict[str, float]] = {}
        self.latest_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.last_seen: Dict[int, float] = {}
        self.offset_sum: Dict[int, float] = {}
        self.offset_count: Dict[int, int] = {}
        self.offset_value: Dict[int, float] = {}
        self.offset_start_time = time.monotonic()
        self.offset_finalized = False
        self.last_cmd: UnitreeCommand = UnitreeCommand()
        self.last_cmd_time: float = 0.0

        ports = list(self.get_parameter("ports").value)
        self.bus0_ids = [int(x) for x in list(self.get_parameter("bus0_ids").value)]
        self.bus1_ids = [int(x) for x in list(self.get_parameter("bus1_ids").value)]

        self.serial_ports: List[SerialPort] = []
        self.threads = []
        for index, port in enumerate(ports):
            try:
                serial_port = SerialPort(port)
            except Exception as exc:
                self.get_logger().error(f"Failed to open {port}: {exc}")
                continue

            self.serial_ports.append(serial_port)
            if index == 0:
                ids = self.bus0_ids
            elif index == 1:
                ids = self.bus1_ids
            else:
                self.get_logger().warn(f"Skipping extra port {port}, no ID mapping")
                continue
            thread = threading.Thread(
                target=self.bus_loop,
                args=(serial_port, port, ids),
                daemon=True,
            )
            thread.start()
            self.threads.append(thread)

        self.timer = self.create_timer(1.0 / self.print_rate, self.print_latest)

    def _normalize_leg_mask(self, mask_values: List[int]) -> List[bool]:
        if len(mask_values) != 4:
            self.get_logger().warn(
                f"leg_feedback_mask expects 4 elements, got {len(mask_values)}. Using all True."
            )
            return [True, True, True, True]
        normalized = []
        for value in mask_values:
            normalized.append(bool(int(value)))
        return normalized

    def on_command_received(self, msg: UnitreeCommand):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def bus_loop(self, serial_port: SerialPort, port: str, ids: List[int]):
        # 获取电机模式和减速比常数
        mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        
        # 发送频率控制
        sleep_interval = 1.0 / max(self.send_rate, 1e-6)

        while rclpy.ok() and not self.stop_event.is_set():
            # 如果只想获取反馈，Kp, Kd, Tau 必须全部为 0
            # 这样电机就处于“失能/透明”状态，但会正常回传位置速度
            kp = 0.0
            kd = 0.0
            pos = 0.0
            tau = 0.0

            for motor_id in ids:
                cmd = MotorCmd()
                data = MotorData()
                
                # 基础配置
                cmd.motorType = MotorType.GO_M8010_6
                data.motorType = MotorType.GO_M8010_6
                cmd.mode = mode
                cmd.id = int(motor_id)
                
                # 关键控制量
                cmd.q = float(pos)
                cmd.dq = 0.0
                cmd.tau = float(tau)
                cmd.kp = float(kp)
                cmd.kd = float(kd)

                try:
                    # 宇树 SDK 的 sendRecv 是阻塞的
                    serial_port.sendRecv(cmd, data)
                    
                    with self.latest_lock:
                        if not self.offset_finalized:
                            current_sum = self.offset_sum.get(motor_id, 0.0) + float(data.q)
                            current_count = self.offset_count.get(motor_id, 0) + 1
                            self.offset_sum[motor_id] = current_sum
                            self.offset_count[motor_id] = current_count
                        self.latest[int(motor_id)] = {
                            "id": int(motor_id),
                            "theta": float(data.q),
                            "omega": float(data.dq),
                            "tau": float(data.tau),
                            "temp": int(data.temp),
                            "merror": int(data.merror),
                        }
                        self.last_seen[int(motor_id)] = time.monotonic()
                except Exception as exc:
                    # 串口错误不应直接 return 退出线程，应尝试重连或跳过
                    self.get_logger().warn(f"Port {port} ID {motor_id} Timeout: {exc}")
                    continue

            time.sleep(sleep_interval)

    def print_latest(self):
        with self.latest_lock:
            if not self.latest:
                return
            if not self.offset_finalized:
                elapsed = time.monotonic() - self.offset_start_time
                if elapsed >= self.offset_window_sec:
                    for motor_id in range(1, 13):
                        if motor_id in self.offset_value:
                            continue
                        count = self.offset_count.get(motor_id, 0)
                        if count > 0:
                            self.offset_value[motor_id] = self.offset_sum[motor_id] / count
                        else:
                            self.offset_value[motor_id] = 0.0
                    self.offset_finalized = True
                    offset_msg = Float64MultiArray()
                    offset_msg.data = [
                        float(self.offset_value.get(motor_id, 0.0))
                        for motor_id in range(1, 13)
                    ]
                    self.offset_pub.publish(offset_msg)
                    self.get_logger().info("Offset window complete, offsets published")
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = []
            joint_state.velocity = []
            now = time.monotonic()
            for motor_id in range(1, 13):
                leg_index = (motor_id - 1) // 3
                data = self.latest.get(motor_id)
                last_seen = self.last_seen.get(motor_id, None)
                is_stale = last_seen is None or (now - last_seen) > self.stale_timeout_sec
                use_feedback = self.leg_feedback_mask[leg_index]
                # Use real feedback per-leg; otherwise fall back to command setpoints.
                if use_feedback:
                    if data is None or is_stale:
                        joint_state.position.append(0.0)
                        joint_state.velocity.append(0.0)
                    else:
                        joint_state.position.append(float(data["theta"]))
                        joint_state.velocity.append(float(data["omega"]))
                else:
                    cmd_stale = (now - self.last_cmd_time) > self.command_stale_timeout_sec
                    cmd_has_data = len(self.last_cmd.q_des) >= 12 and len(self.last_cmd.dq_des) >= 12
                    # Avoid publishing stale or incomplete command data.
                    if cmd_has_data and not cmd_stale:
                        index = motor_id - 1
                        joint_state.position.append(float(self.last_cmd.q_des[index]))
                        joint_state.velocity.append(float(self.last_cmd.dq_des[index]))
                    else:
                        joint_state.position.append(0.0)
                        joint_state.velocity.append(0.0)
            self.joint_state_pub.publish(joint_state)

            lines = ["Unitree feedback (latest):"]
            for motor_id in range(1, 13):
                data = self.latest.get(motor_id)
                last_seen = self.last_seen.get(motor_id, None)
                is_stale = last_seen is None or (now - last_seen) > self.stale_timeout_sec
                if data is None or is_stale:
                    line = f"  id={motor_id} 未正常接收"
                else:
                    if not self.offset_finalized:
                        elapsed = time.monotonic() - self.offset_start_time
                        remaining = max(0.0, self.offset_window_sec - elapsed)
                        line = (
                            f"  id={motor_id} q={data['theta']:.4f} rad "
                            f"dq={data['omega']:.4f} rad/s "
                            f"(calibrating {remaining:.2f}s)"
                        )
                    else:
                        line = (
                            f"  id={motor_id} q={data['theta']:.4f} rad "
                            f"dq={data['omega']:.4f} rad/s"
                        )
                    if self.show_extra:
                        line += (
                            f" tau={data['tau']:.4f} temp={data['temp']}C "
                            f"merror={data['merror']}"
                        )
                lines.append(line)
        self.get_logger().info("\n".join(lines))

    def destroy_node(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=0.5)
        for ser in self.serial_ports:
            try:
                ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FeedbackReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
