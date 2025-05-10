#!/usr/bin/env python3
import numpy as np
import sympy as sp
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mocap_msgs.msg import RigidBodies
from geometry_msgs.msg import Quaternion,Accel
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class RobusLoc(Node):
    def __init__(self):
        """
        Intialize the Localization node.
        Subscribes to the mocap data from OptiTrack
        Creates publisher topics for the estimated odometry and acceleration data.
        """
        super().__init__("robus_localization")
        self.get_logger().info("Localization node has started")

        self.pose_sub = self.create_subscription(RigidBodies, "/pose_modelcars", self.loc_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/ego_odom", 10)
        self.accel_pub = self.create_publisher(Accel, "/ego_accel", 10)
        
        # State vector: [x, y, vx, vy, ax, ay, yaw, yaw_rate]
        self.state = np.zeros((8, 1))
        # Initial Process covariance matrix
        self.P = np.eye(8) * 0.01

        # Measurement noise covariance matrix
        
        # Manual covariance calculation values in IDEAL STATE
        # self.R = np.array([[9.68544654e-09, 0, 0],
        #                    [0, 5.61189430e-09, 0],
        #                    [0, 0, 1.40580618e-07]])
        
        # Assumed measurement covariance values for better Plotjuggler visualization
        self.R = np.array([[0.01, 0, 0],
                           [0, 0.01, 0],
                           [0, 0, 0.01]])

        self.last_time = 0.0

    def loc_callback(self, msg):
        """
        Extended Kalman Filter Implementation to estimate the state of the vehicle.
        Corrects and Predicts the State and Process covariance matrix.
        """
        for rigidbody in msg.rigidbodies:
            # Filtering out the rigidbody with ID 7
            if rigidbody.rigid_body_name != '7':
                continue
            # Computing the time difference(dt) using the header stamp
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_time <= 0.0:
                self.last_time = current_time
                return
            dt = current_time - self.last_time
            self.last_time = current_time
        
            # State Transition Phi Matrix
            Phi = np.eye(8)
            Phi[0, 2] = dt           
            Phi[1, 3] = dt       
            Phi[0, 4] = 0.5 * dt**2 
            Phi[1, 5] = 0.5 * dt**2
            Phi[2, 4] = dt
            Phi[3, 5] = dt
            Phi[6, 7] = dt

            # Process noise covariance matrix
            Q = self.compute_process_noise(Phi, dt)
            # State prediction
            self.state = Phi @ self.state

            # Covariance prediction
            self.P = Phi @ self.P @ Phi.T + Q

            # Measurement vector
            z = np.array([rigidbody.pose.position.x, 
                          rigidbody.pose.position.y, 
                          self.quaternion_to_euler(rigidbody.pose.orientation)[2]]).reshape((3, 1))

            H = np.zeros((3, 8))
            H[0, 0] = 1
            H[1, 1] = 1
            H[2, 6] = 1

            #Kalman Gain
            S = H @ self.P @ H.T + self.R # Innovation of Covariance
            K = self.P @ H.T @ np.linalg.inv(S)

            # Update state and covariance
            y = z - H @ self.state
            self.state = self.state + K @ y
            self.P = (np.eye(8) - K @ H) @ self.P

            self.last_stamp = msg.header.stamp
            self.publish_odom_accel()

    def publish_odom_accel(self):
        """
        Publishes the estimated odometry and acceleration data.
        """
        odom = Odometry()
        accel = Accel()
        odom.header.stamp = self.last_stamp
        odom.header.frame_id = "odom_frame"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = self.yaw_to_quaternion(self.state[6, 0])

        odom.twist.twist.linear.x = float(self.state[2])
        odom.twist.twist.linear.y = float(self.state[3])
        odom.twist.twist.angular.z = float(self.state[7])

        accel.linear.x = float(self.state[4])
        accel.linear.y = float(self.state[5])

        self.odom_pub.publish(odom)
        self.accel_pub.publish(accel)
    
    def compute_process_noise(self,F_k, dt):
        """
        Implementation of Continuous White Noise Model to compute the Process Noise Covariance Matrix.
        Spectral Density(phi_acc, phi_yaw, phi_yawrate) are assumed.
        """
        t = sp.symbols('t')
        phi_acc = 1.0
        phi_yaw = 0.05
        phi_yawrate = 0.2
        Q_c = sp.zeros(8, 8)
        Q_c[4, 4] = phi_acc
        Q_c[5, 5] = phi_acc
        Q_c[6, 6] = phi_yaw
        Q_c[7, 7] = phi_yawrate
        Q_noise = sp.integrate(F_k * Q_c * F_k.T, (t, 0, dt))
        return np.array(Q_noise).astype(np.float64)

    def quaternion_to_euler(self, q):
        """
        Converts quaternion to Euler angles (roll, pitch, yaw).
        """
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return roll, pitch, yaw
    
    def yaw_to_quaternion(self, yaw):
        """
        Converts yaw angle to quaternion.
        """
        q = Quaternion()
        #Assuming flat roll and pitch
        q_array = quaternion_from_euler(0.0, 0.0, yaw)
        q.x = q_array[0]
        q.y = q_array[1]
        q.z = q_array[2]
        q.w = q_array[3]
        return q

def main(args=None):
    rclpy.init(args=args)
    node = RobusLoc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
