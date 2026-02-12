import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from unitree_motor_msgs.msg import UnitreeCommand


class UnitreeCommandPublisher(Node):
    def __init__(self):
        super().__init__("unitree_command_publisher")

        rate_desc = ParameterDescriptor(description="Publish rate (Hz)", dynamic_typing=True)
        self.declare_parameter("publish_rate_hz", 50.0, rate_desc)
        array_desc = ParameterDescriptor(description="12-length float array", dynamic_typing=True)
        self.declare_parameter("q_des", [0.0] * 12, array_desc)
        self.declare_parameter("dq_des", [0.0] * 12, array_desc)
        self.declare_parameter("tau_ff", [0.0] * 12, array_desc)
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("topic", "/unitree_command")

        self.publisher = self.create_publisher(
            UnitreeCommand,
            str(self.get_parameter("topic").value),
            10,
        )

        self.msg = UnitreeCommand()
        self._refresh_msg()

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-6), self.publish_command)

    def _refresh_msg(self):
        q_des = [float(x) for x in list(self.get_parameter("q_des").value)]
        dq_des = [float(x) for x in list(self.get_parameter("dq_des").value)]
        tau_ff = [float(x) for x in list(self.get_parameter("tau_ff").value)]
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)

        if len(q_des) != 12 or len(dq_des) != 12 or len(tau_ff) != 12:
            self.get_logger().error("q_des/dq_des/tau_ff must be length 12")
            return

        self.msg.q_des = q_des
        self.msg.dq_des = dq_des
        self.msg.tau_ff = tau_ff
        self.msg.kp = [kp] * 12
        self.msg.kd = [kd] * 12

    def publish_command(self):
        self._refresh_msg()
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = UnitreeCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
