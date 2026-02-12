import os
import sys
import threading
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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
    queryMotorMode,
)


class M8010Controller(Node):
    def __init__(self):
        super().__init__("unitree_m8010_controller")

        self.gear_ratio = 6.33
        self.q_scale = 1.0 / self.gear_ratio
        self.invert_ids = {2, 3, 7, 10, 11, 12}
        self.pose_offset_by_id = {
            1: -0.35814,
            2: 0.99794,
            3: -2.57956,
            4: 0.35814,
            5: 0.99794,
            6: -2.57956,
            7: 0.35814,
            8: 0.99794,
            9: -2.57956,
            10: -0.35814,
            11: 0.99794,
            12: -2.57956,
        }

        self.declare_parameter("ports", ["/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6THBYZ-if00-port0", 
                                         "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6SV658-if00-port0"])
        self.declare_parameter("bus0_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("bus1_ids", [7, 8, 9, 10, 11, 12])
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_names", [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ])
        self.declare_parameter("joint_state_rate_hz", 100.0)
        self.declare_parameter("send_rate_hz", 100.0)
        self.declare_parameter("offset_window_sec", 1.0)
        self.declare_parameter("offset_topic", "/offset_msg")
        self.declare_parameter("enable_mask", [0] * 12)
        self.declare_parameter("command_topic", "/unitree_command")
        self.declare_parameter("leg_feedback_mask", [0, 0, 0, 1])
        self.declare_parameter("q_delta_topic", "/unitree_q_delta_on_activate")

        self.ports = list(self.get_parameter("ports").value)
        self.bus0_ids = [int(x) for x in list(self.get_parameter("bus0_ids").value)]
        self.bus1_ids = [int(x) for x in list(self.get_parameter("bus1_ids").value)]
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_state_rate = float(self.get_parameter("joint_state_rate_hz").value)
        self.send_rate = float(self.get_parameter("send_rate_hz").value)
        self.offset_window_sec = float(self.get_parameter("offset_window_sec").value)
        self.offset_topic = str(self.get_parameter("offset_topic").value)
        self.enable_mask = [int(x) for x in list(self.get_parameter("enable_mask").value)]
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.leg_feedback_mask = self._normalize_leg_mask(
            list(self.get_parameter("leg_feedback_mask").value)
        )
        self.q_delta_topic = str(self.get_parameter("q_delta_topic").value)

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.offset_pub = self.create_publisher(Float64MultiArray, self.offset_topic, 10)
        self.q_delta_pub = self.create_publisher(Float64MultiArray, self.q_delta_topic, 10)
        # 订阅 QoS 使用 BestEffort，避免与发布端可靠性不匹配导致收不到消息
        command_qos = QoSProfile(depth=10)
        command_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        command_qos.history = QoSHistoryPolicy.KEEP_LAST
        self.command_sub = self.create_subscription(
            UnitreeCommand,
            self.command_topic,
            self.on_command_received,
            command_qos,
        )

        self.latest: Dict[int, Dict[str, float]] = {}
        self.latest_lock = threading.Lock()
        self.last_valid: Dict[int, Dict[str, float]] = {}
        self.offset_value: Dict[int, float] = {}
        self.offset_ready = False
        self.offset_start_time = time.monotonic()
        self.offset_lock = threading.Lock()

        self.last_cmd = UnitreeCommand()
        self.last_cmd.q_des = [0.0] * 12
        self.last_cmd.dq_des = [0.0] * 12
        self.last_cmd.tau_ff = [0.0] * 12
        self.last_cmd.kp = [0.0] * 12
        self.last_cmd.kd = [0.0] * 12
        self.cmd_lock = threading.Lock()
        self.cmd_recv_count = 0
        self.prev_cmd_all_zero = True
        # 记录实际下发到电机的命令（电机侧）
        self.sent_cmd_by_id: Dict[int, Dict[str, float]] = {}
        self.sent_cmd_lock = threading.Lock()

        self.serial_ports: List[SerialPort] = []
        self.threads = []
        for index, port in enumerate(self.ports):
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

        self.timer = self.create_timer(1.0 / self.joint_state_rate, self.publish_joint_state)

    def on_command_received(self, msg: UnitreeCommand):
        with self.cmd_lock:
            self.last_cmd = msg
            self.cmd_recv_count += 1
            # 前几条消息打印采样，便于确认订阅已生效
            if self.cmd_recv_count <= 3:
                try:
                    sample = (
                        float(msg.q_des[0]),
                        float(msg.dq_des[0]),
                        float(msg.kp[0]),
                        float(msg.kd[0]),
                        float(msg.tau_ff[0]),
                    )
                    self.get_logger().info(
                        f"Received /unitree_command sample (id=1): q={sample[0]:.4f} dq={sample[1]:.4f} "
                        f"kp={sample[2]:.4f} kd={sample[3]:.4f} tau={sample[4]:.4f}"
                    )
                except Exception:
                    pass

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

    def bus_loop(self, serial_port: SerialPort, port: str, ids: List[int]):
        # 阶段1/阶段2使用相同的sendRecv读写流程
        mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        sleep_interval = 1.0 / max(self.send_rate, 1e-6)

        while rclpy.ok():
            for motor_id in ids:
                cmd = MotorCmd()
                data = MotorData()

                cmd.motorType = MotorType.GO_M8010_6
                data.motorType = MotorType.GO_M8010_6
                cmd.mode = mode
                cmd.id = int(motor_id)

                # 阶段1：前1秒只发送全0指令读取反馈
                if not self.offset_ready:
                    cmd.q = 0.0
                    cmd.dq = 0.0
                    cmd.kp = 0.0
                    cmd.kd = 0.0
                    cmd.tau = 0.0
                else:
                    # 阶段2：发送控制指令（仅位置加偏移）
                    index = int(motor_id) - 1
                    with self.cmd_lock:
                        q_des = float(self.last_cmd.q_des[index])
                        dq_des = float(self.last_cmd.dq_des[index])
                        tau_ff = float(self.last_cmd.tau_ff[index])
                        kp = float(self.last_cmd.kp[index])
                        kd = float(self.last_cmd.kd[index])
                    mask = 1 if (index < len(self.enable_mask) and self.enable_mask[index] != 0) else 0
                    offset = float(self.offset_value.get(int(motor_id), 0.0))
                    pose_offset = float(self.pose_offset_by_id.get(int(motor_id), 0.0))
                    q_des -= pose_offset
                    direction = -1.0 if int(motor_id) in self.invert_ids else 1.0
                    cmd.q = (q_des + offset) * self.gear_ratio * direction
                    cmd.dq = dq_des * self.gear_ratio * direction
                    # cmd.q = (q_des + offset) * direction
                    # cmd.dq = dq_des  * direction
                    cmd.kp = kp * mask
                    cmd.kd = kd * mask
                    cmd.tau = tau_ff * mask

                # 记录实际将要下发的电机侧命令（包含齿比与方向）
                try:
                    with self.sent_cmd_lock:
                        self.sent_cmd_by_id[int(motor_id)] = {
                            "q": float(cmd.q),
                            "dq": float(cmd.dq),
                            "kp": float(cmd.kp),
                            "kd": float(cmd.kd),
                            "tau": float(cmd.tau),
                        }
                except Exception:
                    pass

                try:
                    serial_port.sendRecv(cmd, data)
                    is_valid = int(data.merror) == 0 and int(data.temp) > 0
                    with self.latest_lock:
                        if is_valid:
                            direction = -1.0 if int(motor_id) in self.invert_ids else 1.0
                            scaled_q = float(data.q) * self.q_scale * direction
                            scaled_dq = float(data.dq) * self.q_scale * direction
                            payload = {
                                "id": int(motor_id),
                                "theta": scaled_q,
                                "omega": scaled_dq,
                                "temp": int(data.temp),
                                "merror": int(data.merror),
                            }
                            self.last_valid[int(motor_id)] = payload
                            self.latest[int(motor_id)] = payload
                        else:
                            if int(motor_id) in self.last_valid:
                                self.latest[int(motor_id)] = self.last_valid[int(motor_id)]

                    if is_valid and not self.offset_ready:
                        with self.offset_lock:
                            if motor_id not in self.offset_value and abs(float(data.q)) > 1e-6:
                                direction = -1.0 if int(motor_id) in self.invert_ids else 1.0
                                self.offset_value[motor_id] = float(data.q) * self.q_scale * direction
                except Exception as exc:
                    self.get_logger().warn(f"Port {port} ID {motor_id} Timeout: {exc}")
                    with self.latest_lock:
                        if int(motor_id) in self.last_valid:
                            self.latest[int(motor_id)] = self.last_valid[int(motor_id)]
                    continue

            time.sleep(sleep_interval)

    def publish_joint_state(self):
        # 阶段1结束时计算偏移并只打印一次
        if not self.offset_ready:
            elapsed = time.monotonic() - self.offset_start_time
            if elapsed >= self.offset_window_sec or len(self.offset_value) >= 12:
                with self.offset_lock:
                    for motor_id in range(1, 13):
                        if motor_id not in self.offset_value:
                            self.offset_value[motor_id] = 0.0
                self.offset_ready = True
                lines = ["Offset (first non-zero) applied:"]
                for motor_id in range(1, 13):
                    lines.append(f"  id={motor_id} offset={self.offset_value[motor_id]:.4f}")
                self.get_logger().info("\n".join(lines))
                offset_msg = Float64MultiArray()
                offset_msg.data = [
                    float(self.offset_value.get(motor_id, 0.0))
                    for motor_id in range(1, 13)
                ]
                self.offset_pub.publish(offset_msg)

        with self.latest_lock:
            with self.cmd_lock:
                cmd_all_zero = True
                for i in range(12):
                    kp = float(self.last_cmd.kp[i]) if i < len(self.last_cmd.kp) else 0.0
                    kd = float(self.last_cmd.kd[i]) if i < len(self.last_cmd.kd) else 0.0
                    tau = float(self.last_cmd.tau_ff[i]) if i < len(self.last_cmd.tau_ff) else 0.0
                    if kp != 0.0 or kd != 0.0 or tau != 0.0:
                        cmd_all_zero = False
                        break

                if self.prev_cmd_all_zero and not cmd_all_zero:
                    delta_msg = Float64MultiArray()
                    delta_msg.data = []
                    for motor_id in range(1, 13):
                        index = motor_id - 1
                        q_des = (
                            float(self.last_cmd.q_des[index])
                            if index < len(self.last_cmd.q_des)
                            else 0.0
                        )
                        data = self.latest.get(motor_id)
                        if data is None:
                            delta = 0.0
                        else:
                            offset = float(self.offset_value.get(motor_id, 0.0))
                            pose_offset = float(self.pose_offset_by_id.get(motor_id, 0.0))
                            fb_q = float(data["theta"]) - offset + pose_offset
                            delta = q_des - fb_q
                        delta_msg.data.append(delta)
                    self.q_delta_pub.publish(delta_msg)

                self.prev_cmd_all_zero = cmd_all_zero

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = []
            joint_state.velocity = []

            for motor_id in range(1, 13):
                leg_index = (motor_id - 1) // 3
                use_feedback = self.leg_feedback_mask[leg_index]
                data = self.latest.get(motor_id)
                if use_feedback:
                    if data is None:
                        joint_state.position.append(0.0)
                        joint_state.velocity.append(0.0)
                    else:
                        offset = float(self.offset_value.get(motor_id, 0.0))
                        pose_offset = float(self.pose_offset_by_id.get(motor_id, 0.0))
                        joint_state.position.append(float(data["theta"]) - offset + pose_offset)
                        joint_state.velocity.append(float(data["omega"]))
                else:
                    index = motor_id - 1
                    with self.cmd_lock:
                        kp = float(self.last_cmd.kp[index]) if index < len(self.last_cmd.kp) else 0.0
                        kd = float(self.last_cmd.kd[index]) if index < len(self.last_cmd.kd) else 0.0
                        tau = float(self.last_cmd.tau_ff[index]) if index < len(self.last_cmd.tau_ff) else 0.0
                        q_des = float(self.last_cmd.q_des[index]) if index < len(self.last_cmd.q_des) else 0.0
                        dq_des = float(self.last_cmd.dq_des[index]) if index < len(self.last_cmd.dq_des) else 0.0

                    if kp == 0.0 and kd == 0.0 and tau == 0.0:
                        pose_offset = float(self.pose_offset_by_id.get(motor_id, 0.0))
                        joint_state.position.append(pose_offset)
                        joint_state.velocity.append(0.0)
                    else:
                        joint_state.position.append(q_des)
                        joint_state.velocity.append(dq_des)

            self.joint_state_pub.publish(joint_state)

            # 实时打印12个电机反馈数据
            lines = ["Unitree feedback:"]
            for motor_id in range(1, 13):
                data = self.latest.get(motor_id)
                if data is None:
                    lines.append(f"  id={motor_id} 未正常接收")
                else:
                    lines.append(
                        f"  id={motor_id} q={data['theta']:.4f} dq={data['omega']:.4f} "
                        f"temp={data['temp']} merror={data['merror']}"
                    )
            with self.cmd_lock:
                cmd_lines = ["Unitree command (q+offset/dq/kp/kd/tau):"]
                for motor_id in range(1, 13):
                    index = motor_id - 1
                    offset = float(self.offset_value.get(motor_id, 0.0))
                    pose_offset = float(self.pose_offset_by_id.get(motor_id, 0.0))
                    q_cmd = float(self.last_cmd.q_des[index]) - pose_offset + offset
                    dq_cmd = float(self.last_cmd.dq_des[index])
                    kp_cmd = float(self.last_cmd.kp[index])
                    kd_cmd = float(self.last_cmd.kd[index])
                    tau_cmd = float(self.last_cmd.tau_ff[index])
                    cmd_lines.append(
                        f"  id={motor_id} q={q_cmd:.4f} dq={dq_cmd:.4f} "
                        f"kp={kp_cmd:.4f} kd={kd_cmd:.4f} tau={tau_cmd:.4f}"
                    )
                # 新增：打印原始接收到的 /unitree_command（不加偏移/不加姿态修正）
                raw_cmd_lines = ["Unitree command RAW (q/dq/kp/kd/tau):"]
                for motor_id in range(1, 13):
                    index = motor_id - 1
                    q_raw = float(self.last_cmd.q_des[index])
                    dq_raw = float(self.last_cmd.dq_des[index])
                    kp_raw = float(self.last_cmd.kp[index])
                    kd_raw = float(self.last_cmd.kd[index])
                    tau_raw = float(self.last_cmd.tau_ff[index])
                    raw_cmd_lines.append(
                        f"  id={motor_id} q={q_raw:.4f} dq={dq_raw:.4f} "
                        f"kp={kp_raw:.4f} kd={kd_raw:.4f} tau={tau_raw:.4f}"
                    )
            # 新增：打印实际下发到电机的命令（包含齿比与方向变换）
            actual_cmd_lines = ["Unitree actual CMD (motor q/dq/kp/kd/tau):"]
            try:
                with self.sent_cmd_lock:
                    for motor_id in range(1, 13):
                        c = self.sent_cmd_by_id.get(motor_id)
                        if c is None:
                            actual_cmd_lines.append(f"  id={motor_id} 未下发")
                        else:
                            actual_cmd_lines.append(
                                f"  id={motor_id} q={c['q']:.4f} dq={c['dq']:.4f} "
                                f"kp={c['kp']:.4f} kd={c['kd']:.4f} tau={c['tau']:.4f}"
                            )
            except Exception:
                pass

            # 新增：打印发送q与反馈q的差值（cmd_q - fb_q）
            delta_lines = ["Unitree q delta (cmd_q - fb_q):"]
            try:
                with self.sent_cmd_lock:
                    for motor_id in range(1, 13):
                        fb = self.latest.get(motor_id)
                        cmd_val = self.sent_cmd_by_id.get(motor_id)
                        if fb is None or cmd_val is None:
                            delta_lines.append(f"  id={motor_id} 无数据")
                        else:
                            delta = float(cmd_val["q"]) - float(fb["theta"])
                            delta_lines.append(f"  id={motor_id} delta={delta:.4f}")
            except Exception:
                pass

            self.get_logger().info("\n".join(lines + cmd_lines + raw_cmd_lines + actual_cmd_lines + delta_lines))

    def destroy_node(self):
        for ser in self.serial_ports:
            try:
                ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = M8010Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
