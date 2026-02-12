import math
import time

import rclpy
from rclpy.node import Node

from unitree_motor_msgs.msg import UnitreeCommand


class SineCommander(Node):
    def __init__(self):
        super().__init__("unitree_sine_commander")
        self.declare_parameter("amplitude", 0.5)
        self.declare_parameter("frequency", 0.25)
        self.declare_parameter("kp", 0.70)
        self.declare_parameter("kd", 0.03)
        self.declare_parameter("enable_sine", False)
        self.declare_parameter("default_q", 0.0)
        self.declare_parameter("default_dq", 0.0)
        self.declare_parameter("default_tau", 0.0)

        self.publisher = self.create_publisher(UnitreeCommand, "unitree_command", 10)
        self.subscriber = self.create_subscription(
            UnitreeCommand,
            "unitree_command",
            self.on_command_received,
            10,
        )
        self.last_cmd = UnitreeCommand()
        self.last_cmd.q_des = [0.0] * 12
        self.last_cmd.dq_des = [0.0] * 12
        self.last_cmd.tau_ff = [0.0] * 12
        self.last_cmd.kp = [float(self.get_parameter("kp").value)] * 12
        self.last_cmd.kd = [float(self.get_parameter("kd").value)] * 12
        self.last_initialized = False
        self.start_time = time.time()
        self.timer = self.create_timer(0.002, self.publish_command)

    def publish_command(self):
        amplitude = float(self.get_parameter("amplitude").value)
        frequency = float(self.get_parameter("frequency").value)
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)

        now = time.time() - self.start_time
        omega = 2.0 * math.pi * frequency
        enable_sine = bool(self.get_parameter("enable_sine").value)
        if enable_sine:
            position = amplitude * math.sin(omega * now)
            velocity = amplitude * omega * math.cos(omega * now)
            default_tau = float(self.get_parameter("default_tau").value)
            self.last_cmd.q_des = [position] * 12
            self.last_cmd.dq_des = [velocity] * 12
            self.last_cmd.tau_ff = [default_tau] * 12
            self.last_cmd.kp = [kp] * 12
            self.last_cmd.kd = [kd] * 12
            self.last_initialized = True
        else:
            if not self.last_initialized:
                position = float(self.get_parameter("default_q").value)
                velocity = float(self.get_parameter("default_dq").value)
                default_tau = float(self.get_parameter("default_tau").value)
                self.last_cmd.q_des = [position] * 12
                self.last_cmd.dq_des = [velocity] * 12
                self.last_cmd.tau_ff = [default_tau] * 12
                self.last_cmd.kp = [kp] * 12
                self.last_cmd.kd = [kd] * 12
                self.last_initialized = True

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


def main():
    rclpy.init()
    node = SineCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
