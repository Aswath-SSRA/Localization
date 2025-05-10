# -*- coding: utf-8 -*-
from robus_localization_pkg.robus_test_localization import RobusLocTest
import rclpy

def test_TC_005():

    # Initialize the ROS 2 context
    rclpy.init()
    robus_loc = RobusLocTest()  # Create an instance of the RobusLoc class
    assert robus_loc.TC_005(True) == "TC_005 passed: accel.linear  written successfully." 
    assert robus_loc.TC_005(False) == "TC_005 failed: accel.linear  was not written."

#Execution : python3 -m pytest TC_005.py