#!/usr/bin/env python

import rospy
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion  #http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
from sensor_msgs.msg import Imu

class Odom_Maker:
    '''
    if map initiated use this object to make odom
    if map changed use Odom_Maker.publishstopper()
    '''

    # get GPS -> make odom inital pose 
    # get imu -> make odom initial roll,pitch,yaw

    def __init__(self):
        rospy.loginfo("Starting Odom_Maker as name_node.")

        gps_subs = rospy.Subscriber("/gps", GPSMessage, self.gps_subs_CB)
        imu_subs = rospy.Subscriber("/imu",Imu,self.imu_CB)
        odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        
        pass

    def gps_subs_CB(self):
        #change gps data to odom tf
        rospy.loginfo("gps_cb_test")
        

        pass

    def imu_CB(self):
        rospy.loginfo("gps_cb_test")
        pass

    def odom_publish(self):
        self.odom_pub.publish()
        rospy.loginfo("odom_pub")

        # publishing odom
        pass

    def publsih_stopper(self):
        #odom publishing stop
        pass


def main():
    rospy.init_node("odom_maker")
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