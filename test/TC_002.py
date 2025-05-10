# -*- coding: utf-8 -*-
from robus_localization_pkg.robus_test_localization import RobusLocTest
import rclpy

def test_TC_002():

    # Initialize the ROS 2 context
    rclpy.init()
    robus_loc = RobusLocTest()  # Create an instance of the RobusLoc class
    assert robus_loc.TC_002(True) == "TC_002 passed: pose.pose.position written successfully."
    assert robus_loc.TC_002(False) == "TC_002 failed: pose.pose.position was not written."

#Execution : python3 -m pytest TC_002.py