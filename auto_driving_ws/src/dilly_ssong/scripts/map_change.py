#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import Imu

class Map_Change:
    def __init__(self):
        rospy.init_node("map_change")
        rospy.Subscriber("/imu",Imu,self.imuCB)

        self.indoor = True
        self.outdoor = False
        self.changed = True

        self.imu_data = Imu()
        self.imu_x = 10
        self.imu_y = 10
        self.imu_z = 10

        rate = rospy.Rate(20)
        os.system('rosrun map_server map_server ~/auto_driving_ws/src/dilly_ssong/maps/indoor.yaml')

        while not rospy.is_shutdown():
            if (self.imu_x == 0 and self.imu_y == 0 and self.imu_z == 0) and self.indoor:
                    # Kill the map server node if it's running
                os.system('pkill map_server')

                # Load a new map file (assuming it's in the current directory)
                os.system('rosrun map_server map_server /auto_driving_ws/src/dilly_ssong/maps/indoor.yaml')

                self.indoor = False
                self.outdoor = True
                rate.sleep()
            
            elif (self.imu_x == 0 and self.imu_y == 0 and self.imu_z == 0) and self.outdoor:
                # Kill the map server node if it's running
                os.system('pkill map_server')

                # Load a new map file (assuming it's in the current directory)
                os.system('rosrun map_server map_server /auto_driving_ws/src/dilly_ssong/maps/outdoor.yaml')

                self.outdoor = False
                self.indoor = True
                rate.sleep()
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