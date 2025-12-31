import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class EKFNode(Node):
    """
    State: x = [px, py, yaw, v, w]^T

    Predict: on /cmd_vel (50 Hz)
    Update:
      - GPS: /gps (5 Hz) -> z = [px, py]
      - IMU yaw: /imu_yaw (50 Hz) -> z = [yaw]
    Publishes:
      /ekf/pose, /ekf/path
    """
    def __init__(self):
        super().__init__("ekf_node")

        self.dt = 0.02

        # mean and covariance
        self.mu = np.zeros((5, 1), dtype=float)
        self.Sigma = np.eye(5) * 0.5

        # Process noise (tune these!)
        self.R = np.diag([0.02, 0.02, 0.01, 0.10, 0.10]) ** 2

        # Measurement noise
        self.Q_gps = np.diag([0.10, 0.10]) ** 2      # meters^2
        self.Q_yaw = np.array([[0.03 ** 2]])         # rad^2

        self.last_cmd = Twist()
        self.have_cmd = False

        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.sub_gps = self.create_subscription(PoseStamped, "/gps", self.on_gps, 10)
        self.sub_yaw = self.create_subscription(Float64, "/imu_yaw", self.on_yaw, 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/ekf/pose", 10)
        self.path_pub = self.create_publisher(Path, "/ekf/path", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.timer = self.create_timer(self.dt, self.predict_step)

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.have_cmd = True

    def predict_step(self):
        if not self.have_cmd:
            return

        v_cmd = float(self.last_cmd.linear.x)
        w_cmd = float(self.last_cmd.angular.z)

        px, py, yaw, v, w = self.mu.flatten().tolist()

        # Use commanded v,w as next v,w (simple model)
        v_next = v_cmd
        w_next = w_cmd

        # Nonlinear motion model
        px_next = px + v * math.cos(yaw) * self.dt
        py_next = py + v * math.sin(yaw) * self.dt
        yaw_next = wrap(yaw + w * self.dt)

        self.mu = np.array([[px_next], [py_next], [yaw_next], [v_next], [w_next]])

        # Jacobian G = d g / d x
        G = np.eye(5)
        G[0, 2] = -v * math.sin(yaw) * self.dt
        G[0, 3] =  math.cos(yaw) * self.dt
        G[1, 2] =  v * math.cos(yaw) * self.dt
        G[1, 3] =  math.sin(yaw) * self.dt
        G[2, 4] =  self.dt

        # Covariance prediction
        self.Sigma = G @ self.Sigma @ G.T + self.R

        self.publish_estimate()

    def on_gps(self, msg: PoseStamped):
        z = np.array([[msg.pose.position.x], [msg.pose.position.y]])

        # h(x) = [px, py]
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        h = np.array([[self.mu[0, 0]], [self.mu[1, 0]]])

        self.ekf_update(z, h, H, self.Q_gps, angle_index=None)
        self.publish_estimate()

    def on_yaw(self, msg: Float64):
        z = np.array([[wrap(float(msg.data))]])

        # h(x) = [yaw]
        H = np.zeros((1, 5))
        H[0, 2] = 1.0
        h = np.array([[self.mu[2, 0]]])

        # yaw needs wrapping in innovation
        self.ekf_update(z, h, H, self.Q_yaw, angle_index=0)
        self.publish_estimate()

    def ekf_update(self, z, h, H, Q, angle_index=None):
        # innovation
        y = z - h
        if angle_index is not None:
            y[angle_index, 0] = wrap(y[angle_index, 0])

        S = H @ self.Sigma @ H.T + Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        self.mu = self.mu + K @ y
        self.mu[2, 0] = wrap(self.mu[2, 0])  # keep yaw wrapped

        I = np.eye(self.Sigma.shape[0])
        self.Sigma = (I - K @ H) @ self.Sigma

    def publish_estimate(self):
        # PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = float(self.mu[0, 0])
        msg.pose.pose.position.y = float(self.mu[1, 0])
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = float(self.mu[2, 0])
        msg.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        msg.pose.pose.orientation.w = float(np.cos(yaw / 2.0))

        # fill covariance (6x6): we only map x,y,yaw into it (others left large)
        cov = np.eye(6) * 999.0
        cov[0, 0] = float(self.Sigma[0, 0])
        cov[1, 1] = float(self.Sigma[1, 1])
        cov[5, 5] = float(self.Sigma[2, 2])  # yaw in index 5 for ROS pose covariance
        msg.pose.covariance = cov.flatten().tolist()

        self.pose_pub.publish(msg)

        # Path
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(ps)

        # keep path from growing forever
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses = self.path_msg.poses[-5000:]

        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
