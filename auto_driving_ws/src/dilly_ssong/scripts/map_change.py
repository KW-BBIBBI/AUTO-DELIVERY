#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
class Map_Change:
    def __init__(self):
        rospy.init_node("map_change")
        rospy.Subscriber("/imu",Imu,self.imuCB)
        self.flag_pub = rospy.Publisher("/flag", Bool, queue_size=10)

        self.flag = Bool()
        self.flag.data = True

        self.indoor = True
        self.outdoor = False
        self.changed = True

        self.imu_data = Imu()
        self.imu_x = 10
        self.imu_y = 10
        self.imu_z = 10

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if (self.imu_x == 0 and self.imu_y == 0 and self.imu_z == 0) and self.indoor:
                os.system("gnome-terminal -- bash -c 'roslaunch dilly_ssong outdoor.launch'")
                self.indoor = False
                self.outdoor = True
                self.flag.data = False
                rate.sleep()

            elif (self.imu_x == 0 and self.imu_y == 0 and self.imu_z == 0) and self.outdoor:
                os.system("gnome-terminal -- bash -c 'roslaunch dilly_ssong indoor.launch'")
                self.outdoor = False
                self.indoor = True
                self.flag.data = True
                rate.sleep()

            self.flag_pub.publish(self.flag.data)
            rate.sleep()

    def imuCB(self,data):
        self.imu_data = data

        self.imu_x = self.imu_data.linear_acceleration.x
        self.imu_y = self.imu_data.linear_acceleration.y
        self.imu_z = self.imu_data.linear_acceleration.z

def main():
    try:
        map_change=Map_Change()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
