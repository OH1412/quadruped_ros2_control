import rclpy
from rclpy.node import Node

from unitree_motor_msgs.msg import UnitreeCommand


class FixedAngleCommander(Node):
    def __init__(self):
        super().__init__("unitree_fixed_angle_commander")
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("kp", 15.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("q_des", [0.0] * 12)
        self.declare_parameter("dq_des", [0.0] * 12)
        self.declare_parameter("tau_ff", [0.0] * 12)

        self.publisher = self.create_publisher(UnitreeCommand, "unitree_command", 10)

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

    def publish_command(self):
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        q_des = list(self.get_parameter("q_des").value)
        dq_des = list(self.get_parameter("dq_des").value)
        tau_ff = list(self.get_parameter("tau_ff").value)

        if len(q_des) != 12 or len(dq_des) != 12 or len(tau_ff) != 12:
            self.get_logger().error("q_des/dq_des/tau_ff must be length 12")
            return

        msg = UnitreeCommand()
        msg.q_des = q_des
        msg.dq_des = dq_des
        msg.tau_ff = tau_ff
        msg.kp = [kp] * 12
        msg.kd = [kd] * 12

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FixedAngleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
