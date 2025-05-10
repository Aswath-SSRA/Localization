# -*- coding: utf-8 -*-
from robus_localization_pkg.robus_test_localization import RobusLocTest
import rclpy

def test_TC_003():

    # Initialize the ROS 2 context
    rclpy.init()
    robus_loc = RobusLocTest()  # Create an instance of the RobusLoc class
    assert robus_loc.TC_003(True) == "TC_003 passed: twist.twist.linear written successfully."
    assert robus_loc.TC_003(False) == "TC_003 failed: twist.twist.linear was not written."

#Execution : python3 -m pytest TC_003.py