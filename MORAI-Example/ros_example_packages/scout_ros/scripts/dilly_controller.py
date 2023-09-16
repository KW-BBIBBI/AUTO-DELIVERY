#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd, SkidSteer6wUGVStatus
from geometry_msgs.msg import Twist

class dilly_controller :

    def __init__(self):
        rospy.init_node('dilly_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd',SkidSteer6wUGVCtrlCmd, queue_size=1)
        rospy.Subscriber('/6wheel_skid_status', SkidSteer6wUGVStatus, self.statusCB)
        dilly_cmd_vel_msg = SkidSteer6wUGVCtrlCmd()

        rate = rospy.Rate(20) 
        while not rospy.is_shutdown():
            dilly_cmd_vel_msg.cmd_type = 1
        
            # # 직진
            dilly_cmd_vel_msg.Forward_input = True
            
            # # 후진
            # dilly_cmd_vel_msg.cmd_type = 1
            # dilly_cmd_vel_msg.Backward_input = True

            # # 좌회전
           
            # # 우회전

            # # 제자리 회전
            self.cmd_vel_pub.publish(dilly_cmd_vel_msg)
            rate.sleep()



    def statusCB(self,msg):
        #선속도, 각속도
        print(msg.left_front_wheel_rpm)
        print(msg.acceleration, msg.position)

if __name__ == '__main__':
    
    test=dilly_controller()



