import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


GRAVITY_M_S2 = 9.80665


class LivoxImuConverter(Node):
    """Scale Livox IMU acceleration (reported in g) to m/s^2 and republish."""

    def __init__(self) -> None:
        super().__init__('livox_imu_converter')

        self.input_topic = self.declare_parameter('input_topic', '/livox/imu').value
        self.output_topic = self.declare_parameter('output_topic', '/imu').value
        self.acc_scale = float(self.declare_parameter('acc_scale', GRAVITY_M_S2).value)
        self.odom_topic = self.declare_parameter('odom_topic', '/LIVO2/imu_propagate').value
        self._latest_orientation = None
        self._warned_missing_odom = False

        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(Imu, self.input_topic, self._callback, qos)
        self.publisher = self.create_publisher(Imu, self.output_topic, qos)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos)

        self.get_logger().info(
            f"Republishing IMU from {self.input_topic} to {self.output_topic} with acc scale {self.acc_scale:.5f}"
        )

    def _callback(self, msg: Imu) -> None:
        out_msg = Imu()
        out_msg.header = msg.header

        out_msg.angular_velocity = msg.angular_velocity
        out_msg.angular_velocity_covariance = list(msg.angular_velocity_covariance)

        out_msg.linear_acceleration.x = msg.linear_acceleration.x * self.acc_scale
        out_msg.linear_acceleration.y = msg.linear_acceleration.y * self.acc_scale
        out_msg.linear_acceleration.z = msg.linear_acceleration.z * self.acc_scale
        out_msg.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)

        if self._latest_orientation is None:
            if not self._warned_missing_odom:
                self._warned_missing_odom = True
                self.get_logger().warn(
                    f"No odometry received on {self.odom_topic}; skip publishing IMU data until it arrives."
                )
            return

        out_msg.orientation = self._latest_orientation
        out_msg.orientation_covariance = list(msg.orientation_covariance)

        self.publisher.publish(out_msg)

    def _odom_callback(self, msg: Odometry) -> None:
        # store the latest quaternion so imu callback can reuse it
        self._latest_orientation = msg.pose.pose.orientation
        self._warned_missing_odom = False


def main() -> None:
    rclpy.init()
    node = LivoxImuConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
