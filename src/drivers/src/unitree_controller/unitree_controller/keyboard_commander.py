import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node

from unitree_motor_msgs.msg import UnitreeCommand


class KeyboardCommander(Node):
    def __init__(self):
        super().__init__("unitree_keyboard_commander")
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("step_q", 0.05)
        self.declare_parameter("step_dq", 0.1)
        self.declare_parameter("step_tau", 0.1)
        self.declare_parameter("step_kp", 1.0)
        self.declare_parameter("step_kd", 0.1)
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("kd", 0.5)

        self.publisher = self.create_publisher(UnitreeCommand, "unitree_command", 10)
        self.subscriber = self.create_subscription(
            UnitreeCommand,
            "unitree_command",
            self.on_command_received,
            10,
        )
        self.lock = threading.Lock()

        self.q = [0.0] * 12
        self.dq = [0.0] * 12
        self.tau = [0.0] * 12
        self.kp = [float(self.get_parameter("kp").value)] * 12
        self.kd = [float(self.get_parameter("kd").value)] * 12

        self.selected_index = -1

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.print_help()

    def print_help(self):
        self.get_logger().info("Keyboard control: ")
        self.get_logger().info("  w/s: q +/-,  e/d: dq +/-,  t/g: tau +/-,  p/l: kp +/-,  o/k: kd +/-")
        self.get_logger().info("  [ / ]: select joint,  m: toggle all joints,  0: zero all,  q: quit")
        self.get_logger().info("  Selected joint: all" if self.selected_index == -1 else f"  Selected joint: {self.selected_index}")

    def apply_delta(self, target, delta):
        if self.selected_index == -1:
            for i in range(12):
                target[i] += delta
        else:
            target[self.selected_index] += delta

    def zero_all(self):
        for i in range(12):
            self.q[i] = 0.0
            self.dq[i] = 0.0
            self.tau[i] = 0.0

    def publish_command(self):
        with self.lock:
            msg = UnitreeCommand()
            msg.q_des = list(self.q)
            msg.dq_des = list(self.dq)
            msg.tau_ff = list(self.tau)
            msg.kp = list(self.kp)
            msg.kd = list(self.kd)
        self.publisher.publish(msg)

    def on_command_received(self, msg: UnitreeCommand):
        lines = ["Received UnitreeCommand:"]
        for i in range(12):
            lines.append(
                f"  motor[{i}] q={msg.q_des[i]:.4f} dq={msg.dq_des[i]:.4f} "
                f"tau={msg.tau_ff[i]:.4f} kp={msg.kp[i]:.2f} kd={msg.kd[i]:.2f}"
            )
        self.get_logger().info("\n".join(lines))

    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                key = sys.stdin.read(1)
                if not key:
                    time.sleep(0.01)
                    continue
                with self.lock:
                    if key == "w":
                        self.apply_delta(self.q, float(self.get_parameter("step_q").value))
                    elif key == "s":
                        self.apply_delta(self.q, -float(self.get_parameter("step_q").value))
                    elif key == "e":
                        self.apply_delta(self.dq, float(self.get_parameter("step_dq").value))
                    elif key == "d":
                        self.apply_delta(self.dq, -float(self.get_parameter("step_dq").value))
                    elif key == "t":
                        self.apply_delta(self.tau, float(self.get_parameter("step_tau").value))
                    elif key == "g":
                        self.apply_delta(self.tau, -float(self.get_parameter("step_tau").value))
                    elif key == "p":
                        self.apply_delta(self.kp, float(self.get_parameter("step_kp").value))
                    elif key == "l":
                        self.apply_delta(self.kp, -float(self.get_parameter("step_kp").value))
                    elif key == "o":
                        self.apply_delta(self.kd, float(self.get_parameter("step_kd").value))
                    elif key == "k":
                        self.apply_delta(self.kd, -float(self.get_parameter("step_kd").value))
                    elif key == "[":
                        if self.selected_index == -1:
                            self.selected_index = 0
                        else:
                            self.selected_index = max(0, self.selected_index - 1)
                        self.print_help()
                    elif key == "]":
                        if self.selected_index == -1:
                            self.selected_index = 0
                        else:
                            self.selected_index = min(11, self.selected_index + 1)
                        self.print_help()
                    elif key == "m":
                        self.selected_index = -1 if self.selected_index != -1 else 0
                        self.print_help()
                    elif key == "0":
                        self.zero_all()
                    elif key == "q":
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = KeyboardCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
