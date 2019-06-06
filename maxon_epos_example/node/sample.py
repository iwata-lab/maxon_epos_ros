#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# FileName:     sample
# CreatedDate:  2019-06-05 18:25:22
#

import rospy
from std_msgs.msg import Float32MultiArray


if __name__ == "__main__":
    rospy.init_node("sample", anonymous=True)
    pub = rospy.Publisher("/maxon_bringup/all_position", Float32MultiArray, queue_size=1)
    pub.publish(Float32MultiArray(data=[10000, -20000]))
