#!/usr/bin/env python
import rospy
from morai_msgs.msg import GPSmessage



class map_flag_pub:
    def __init__(self):
        rospy.init_node("map_flag")
        rospy.loginfo("Starting map_flag_pub as map_flag.")
        
        rospy.Subscriber("/gps",GPSmessage,self.gpsCB)
        self.map_flag_publisher = rospy.Publisher("/flag",bool,queue_size=1)
        
        self.gps_data_curr = None
        self.gps_data_prev = None
        self.changed = False
        
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
         if self.gps_data_prev is not None:
            if self.gps_data_curr - self.gps_data_prev > 0.00001:   # 값은 나중에 재설정 필요함
                self.changed = True
            else:
                self.changed = False
        else:
            self.gps_data_prev = self.gps_data_curr

    
    def gpsCB(self,data):
        self.gps_data_curr=data    

    def change_status(self):
        if self.gps_data_curr == 0:
            self.map_flag_publisher 


if __name__ == "__main__":
    map_flag = map_flag_pub()
    rospy.spin()