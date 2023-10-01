#!/usr/bin/env python
import rospy

from morai_msgs.msg import GPSmessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import pyproj
import tf
import numpy as np

from math import cos,sin,sqrt,pow,atan2,pi


class Odom_tf_build:
    def __init__(self):
        rospy.init_node("make_odom_tf")
        rospy.loginfo("Starting Odom_tf_build as name_node.")

        rospy.Subscriber("/gps",GPSmessage, self.gpsCB)
        rospy.Subscriber("/imu",Imu, self.imuCB)
        self.odom_pub=rospy.Publisher("odom",Odometry,queue_size=10)

        self.odom=Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id= "base_link"

        # flag
        self.is_gps = False
        self.is_imu = False

        self.x_offset = 0
        self.y_offset = 0

        # convert gps to UTM
        self.proj_UTM = pyproj.Proj(proj="utm",zone=52,ellps="WGS84",preserve_units=False)

        # 30Hz
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            if self.is_gps and self.is_imu:
                self.MakeOdomMsg()
                self.MakeOdomTF()
                rate.sleep()

    def gpsCB(self, data):
        self.xy_data = self.proj_UTM(data.longitude, data.latitude)
        self.is_gps = True
        

    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.is_imu = True
    
    def MakeOdomMsg(self):
        quaternion = tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2])
        self.odom.pose.pose.position.y=self.xy_zone[1] - self.y_offset
        self.odom.pose.pose.position.z=0
        self.odom.pose.pose.orientation.x=quaternion[0]
        self.odom.pose.pose.orientation.y=quaternion[1]
        self.odom.pose.pose.orientation.z=quaternion[2]
        self.odom.pose.pose.orientation.w=quaternion[3]
        self.odom_pub.publish(self.odom)


    def MakeOdomTF(self):
        br =tf.TransformBroadcaster()
        position_x = self.xy_zone[0] - self.x_offset
        position_y = self.xy_zone[1] - self.y_offset
        
        br.sendTransform((position_x, position_y, 0),
                        tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2]),
                        rospy.Time.now(),
                        "base_link",
                        "odom")


if __name__ == "__main__":
    name_node = Odom_tf_build()
    rospy.spin()