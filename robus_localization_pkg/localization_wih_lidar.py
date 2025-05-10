import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mocap_msgs.msg import RigidBodies
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion

class EKFLocalizer(Node):
    def __init__(self):
        super().__init__('ekf_localizer_node')

        self.pose_sub = self.create_subscription(RigidBodies, "/pose_modelcars", self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/ego_odom", 10)

        # Timer to call the predict function at a fixed rate(20Hz)
        self.timer = self.create_timer(0.05, self.predict)

        self.state = np.zeros((6, 1))  # [x, y, vx, vy, yaw, yaw_rate]
        self.P = np.eye(6) * 0.1

        self.Q = np.diag([0.02, 0.02, 0.1, 0.1, 0.01, 0.01])  # Process noise
        self.R_pose = np.diag([0.02, 0.02, 0.01])  # Pose measurement noise
        self.R_lidar = np.diag([0.1, 0.05])  # Lidar measurement noise (range, bearing)

        self.last_time = self.get_clock().now()

        # Traffic light landmark positions
        self.landmarks = np.array([
            [5.1, 3.1], [5.1, 1.9], [2.9, 1.9],
            [2.9, 3.1], [1.1, 3.1], [1.1, 1.9]
        ])

    def predict(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        x, y, vx, vy, yaw, yaw_rate = self.state.flatten()

        # Prediction Model f(x, u)
        x_pred = x + vx * dt
        y_pred = y + vy * dt
        vx_pred = vx
        vy_pred = vy
        yaw_pred = yaw + yaw_rate * dt
        yaw_rate_pred = yaw_rate

        self.state = np.array([[x_pred], [y_pred], [vx_pred], [vy_pred], [yaw_pred], [yaw_rate_pred]])

        # Jacobian of the motion model F
        F = np.eye(6)
        F[0, 2] = dt
        F[1, 3] = dt
        F[4, 5] = dt

        # Update state covariance
        self.P = F @ self.P @ F.T + self.Q

        self.last_time = now

    def pose_callback(self, msg):
        for rb in msg.rigidbodies:
            if rb.rigid_body_name != '7':
                continue

            px = rb.pose.position.x
            py = rb.pose.position.y
            quat = rb.pose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            z = np.array([[px], [py], [yaw]])

            H = np.zeros((3, 6))
            H[0, 0] = 1
            H[1, 1] = 1
            H[2, 4] = 1

            y = z - H @ self.state
            S = H @ self.P @ H.T + self.R_pose
            K = self.P @ H.T @ np.linalg.inv(S)

            self.state = self.state + K @ y
            self.P = (np.eye(6) - K @ H) @ self.P

            self.publish_odom()

    def lidar_callback(self, scan):
        # Simulate detection of closest known landmark from scan
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = np.array(scan.ranges)

        for i, r in enumerate(ranges):
            if np.isinf(r) or r < 0.01:
                continue

            angle = angle_min + i * angle_increment
            # Transform lidar detection to global coordinates
            lx = self.state[0, 0] + r * np.cos(self.state[4, 0] + angle)
            ly = self.state[1, 0] + r * np.sin(self.state[4, 0] + angle)

            # Find nearest landmark
            dists = np.linalg.norm(self.landmarks - np.array([lx, ly]), axis=1)
            min_idx = np.argmin(dists)

            if dists[min_idx] < 0.5:  # Association threshold
                landmark = self.landmarks[min_idx]
                dx = landmark[0] - self.state[0, 0]
                dy = landmark[1] - self.state[1, 0]
                range_meas = np.sqrt(dx ** 2 + dy ** 2)
                bearing_meas = np.arctan2(dy, dx) - self.state[4, 0]

                z = np.array([[range_meas], [bearing_meas]])

                # Measurement prediction
                px, py, yaw = self.state[0, 0], self.state[1, 0], self.state[4, 0]
                dx = landmark[0] - px
                dy = landmark[1] - py
                r_pred = np.sqrt(dx ** 2 + dy ** 2)
                b_pred = np.arctan2(dy, dx) - yaw

                z_pred = np.array([[r_pred], [b_pred]])

                # Jacobian H
                H = np.zeros((2, 6))
                q = dx ** 2 + dy ** 2
                H[0, 0] = -(dx) / np.sqrt(q)
                H[0, 1] = -(dy) / np.sqrt(q)
                H[1, 0] = dy / q
                H[1, 1] = -dx / q
                H[1, 4] = -1

                y = z - z_pred
                S = H @ self.P @ H.T + self.R_lidar
                K = self.P @ H.T @ np.linalg.inv(S)

                self.state = self.state + K @ y
                self.P = (np.eye(6) - K @ H) @ self.P
                break

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(self.state[4, 0])
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = float(self.state[2])
        odom.twist.twist.linear.y = float(self.state[3])
        odom.twist.twist.angular.z = float(self.state[5])

        self.odom_pub.publish(odom)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(yaw / 2.0)
        q.w = np.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
