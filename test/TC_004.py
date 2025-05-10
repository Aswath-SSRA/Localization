# -*- coding: utf-8 -*-
from robus_localization_pkg.robus_test_localization import RobusLocTest
import rclpy

def test_TC_004():

    # Initialize the ROS 2 context
    rclpy.init()
    robus_loc = RobusLocTest()  # Create an instance of the RobusLoc class
    assert robus_loc.TC_004(True) == "TC_004 passed: twist.twist.angular written successfully." 
    assert robus_loc.TC_004(False) == "TC_004 failed: twist.twist.angular was not written."

#Execution : python3 -m pytest TC_004.py