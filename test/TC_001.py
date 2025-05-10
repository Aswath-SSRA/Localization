# -*- coding: utf-8 -*-
from robus_localization_pkg.robus_test_localization import RobusLocTest
import rclpy

def test_TC_001():

    # Initialize the ROS 2 context
    rclpy.init()
    robus_loc = RobusLocTest()  # Create an instance of the RobusLoc class
    assert robus_loc.TC_001(2.0) == "TC_001 failed: Measurement frequency is invalid."
    assert robus_loc.TC_001(33.3) == "TC_001 passed: Measurement frequency is valid."

#Execution : python3 -m pytest TC_001.py