#!/usr/bin/env python
import rospy
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion

class Odom_Maker:
    # get GPS -> make odom inital pose 
    # get imu -> make odom initial roll,pitch,yaw

    def __init__(self):
        rospy.loginfo("Starting Odom_Maker as name_node.")

        gps_subs=rospy.Subscriber("/gps", GPSMessage, self.gps_subs_CB)
        odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
         

        pass

    def gps_subs_CB(self):
        #change

        pass

    def odom_publish(self):
        # publishing odom
        pass

def main():
    rospy.init_node("odom maker")
    kimheosu = Odom_Maker()
    try:
        rate = rospy.Rate(50) # 50hz
        while not rospy.is_shutdown():
            kimheosu.odom_publish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass



if __name__ == "__main__":
    main()