import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Path

class SimNode(Node):
    """
    Publishes:
      /cmd_vel (Twist)              at 50 Hz
      /truth/pose (PoseStamped)     at 50 Hz
      /truth/path (Path)            at 50 Hz
      /gps (PoseStamped)            at 5 Hz
      /imu_yaw (Float64)            at 50 Hz
    """
    def __init__(self):
        super().__init__("sim_node")

        self.dt = 0.02          # 50 Hz
        self.gps_dt = 0.2       # 5 Hz

        # Ground truth state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Motion profile (circle-ish)
        self.v_cmd = 0.4
        self.w_cmd = 0.6

        # Sensor noise std dev
        self.gps_std = 0.10     # meters
        self.yaw_std = 0.03     # rad

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.truth_pub = self.create_publisher(PoseStamped, "/truth/pose", 10)
        self.truth_path_pub = self.create_publisher(Path, "/truth/path", 10)
        self.gps_pub = self.create_publisher(PoseStamped, "/gps", 10)
        self.imu_yaw_pub = self.create_publisher(Float64, "/imu_yaw", 10)

        self.truth_path = Path()
        self.truth_path.header.frame_id = "map"

        self.t = 0.0
        self.gps_acc = 0.0

        self.timer = self.create_timer(self.dt, self.step)

    def step(self):
        # publish cmd
        cmd = Twist()
        cmd.linear.x = self.v_cmd
        cmd.angular.z = self.w_cmd
        self.cmd_pub.publish(cmd)

        # integrate truth (unicycle kinematics)
        self.x += self.v_cmd * math.cos(self.th) * self.dt
        self.y += self.v_cmd * math.sin(self.th) * self.dt
        self.th = self.wrap(self.th + self.w_cmd * self.dt)
        self.t += self.dt

        # publish truth pose
        truth_pose = self.make_pose("map", self.x, self.y, self.th)
        self.truth_pub.publish(truth_pose)
        
        # Append to truth path
        self.truth_path.header.stamp = self.get_clock().now().to_msg()
        self.truth_path.poses.append(truth_pose)
        self.truth_path_pub.publish(self.truth_path)

        # IMU yaw (noisy)
        yaw_msg = Float64()
        yaw_msg.data = self.wrap(self.th + random.gauss(0.0, self.yaw_std))
        self.imu_yaw_pub.publish(yaw_msg)

        # GPS at lower rate
        self.gps_acc += self.dt
        if self.gps_acc >= self.gps_dt:
            self.gps_acc = 0.0
            gx = self.x + random.gauss(0.0, self.gps_std)
            gy = self.y + random.gauss(0.0, self.gps_std)
            self.gps_pub.publish(self.make_pose("map", gx, gy, self.th))

    def make_pose(self, frame_id, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        # Convert yaw to quaternion (rotation around z-axis)
        msg.pose.orientation.z = float(math.sin(yaw / 2.0))
        msg.pose.orientation.w = float(math.cos(yaw / 2.0))
        return msg

    @staticmethod
    def wrap(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

def main():
    rclpy.init()
    node = SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
