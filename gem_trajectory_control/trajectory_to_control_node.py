import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped


class TrajectoryToControlNode(Node):
    def __init__(self):
        super().__init__("trajectory_to_control_node")

        self.path = None
        self.odom = None
        self.target_speed = 0.0

        self.wheelbase = 1.75
        self.lookahead_distance = 2.0
        self.max_steering_angle = 0.45
        self.max_speed = 1.5

        self.create_subscription(
            Path,
            "/planning/local_path",
            self.path_callback,
            10,
        )

        self.create_subscription(
            Float32,
            "/planning/target_speed",
            self.speed_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            "/odometry",
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            "/cmd_ackermann",
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Trajectory-to-control node initialized.")

    def path_callback(self, msg: Path):
        self.path = msg

    def speed_callback(self, msg: Float32):
        self.target_speed = msg.data

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def timer_callback(self):
        if self.odom is None or self.path is None or len(self.path.poses) == 0:
            self.publish_stop()
            return

        steering_angle = self.compute_pure_pursuit_steering()

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = max(0.0, min(self.target_speed, self.max_speed))

        self.cmd_pub.publish(cmd)

    def compute_pure_pursuit_steering(self):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        yaw = self.quaternion_to_yaw(self.odom.pose.pose.orientation)

        target = self.find_lookahead_point(x, y)

        if target is None:
            return 0.0

        tx = target.pose.position.x
        ty = target.pose.position.y

        dx = tx - x
        dy = ty - y

        # Transform target into vehicle frame.
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if local_x <= 0.0:
            return 0.0

        alpha = math.atan2(local_y, local_x)

        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            self.lookahead_distance,
        )

        steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering),
        )

        return steering

    def find_lookahead_point(self, x, y):
        best_pose = None

        for pose in self.path.poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - x, py - y)

            if dist >= self.lookahead_distance:
                best_pose = pose
                break

        if best_pose is None:
            best_pose = self.path.poses[-1]

        return best_pose

    def publish_stop(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        self.cmd_pub.publish(cmd)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()