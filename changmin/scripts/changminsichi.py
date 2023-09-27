#! /usr/bin/python3

import rospy

from morai_msgs.msg import GPSMessage

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import pyproj
import tf
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi

class MakeOdom:
    def __init__(self):
        rospy.init_node('make_odom', anonymous=True)
        
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.imuCB)
        self.odom_pub = rospy.Publisher('odom_test',Odometry, queue_size=1)

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.is_gps = False
        self.is_imu = False

        # self.x_offset = 434.32594799867366
        # self.y_offset = -100.06033325288445
        self.x_offset = 334209.0789507285
        self.y_offset = 4143042.501430065
        
        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.is_gps and self.is_imu:    
                self.makeOdomMsg()
                self.makeOdomTF()
                rate.sleep()

    def makeOdomTF(self):
        br =tf.TransformBroadcaster()
        position_x = self.xy_zone[0] - self.x_offset
        position_y = self.xy_zone[1] - self.y_offset
        
        br.sendTransform((position_x, position_y, 0),
                        tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2]),
                        rospy.Time.now(),
                        "base_link",
                        "odom")
                        
    def makeOdomMsg(self):
        quaternion = tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2])
        self.odom.pose.pose.position.x=self.xy_zone[0] - self.x_offset
        self.odom.pose.pose.position.y=self.xy_zone[1] - self.y_offset
        self.odom.pose.pose.position.z=0
        self.odom.pose.pose.orientation.x=quaternion[0]
        self.odom.pose.pose.orientation.y=quaternion[1]
        self.odom.pose.pose.orientation.z=quaternion[2]
        self.odom.pose.pose.orientation.w=quaternion[3]
        self.odom_pub.publish(self.odom)

    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        self.is_gps = True

    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        # self.heading = self.euler_data[2]
        self.is_imu = True

def main():
    try:
        makeodom=MakeOdom()    
    except rospy.ROSInterruptException:
        pass
    
if __name__ == '__main__':
    main()