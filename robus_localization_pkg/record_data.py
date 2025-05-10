import rclpy
from rclpy.node import Node
from mocap_msgs.msg import RigidBodies
import csv
import os
import math
from datetime import datetime


class OptiTrackLogger(Node):
    def __init__(self):
        super().__init__('optitrack_logger')
        self.subscription = self.create_subscription(RigidBodies,'/pose_modelcars',self.listener_callback,10)
        self.last_time = None

        # Create or overwrite CSV
        self.file_path= "/home/aswath/robus_localization/src/robus_loc/resource"
        os.makedirs(self.file_path, exist_ok=True)
        self.filename = os.path.join(self.file_path, f"optitrack_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "X", "Y", "Yaw"])
        self.get_logger().info("OptiTrack Logger Node has started")

    def listener_callback(self, msg):
        for rigidbody in msg.rigidbodies:
            if rigidbody.rigid_body_name == '7':
                # Extract time
                current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

                # Extract position
                x = rigidbody.pose.position.x
                y = rigidbody.pose.position.y

                # Convert quaternion to yaw
                q = rigidbody.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y**2 + q.z**2))

                # Save to CSV
                with open(self.filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([current_time, x, y, yaw])

                self.get_logger().info(f"Logging started. Saving to {self.file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
