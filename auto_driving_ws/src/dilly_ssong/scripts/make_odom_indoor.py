#! /usr/bin/python3

import rospy

from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import pyproj
import tf
import numpy as np

from std_msgs.msg import Bool
from math import cos,sin,sqrt,pow,atan2,pi

class MakeOdom:
    def __init__(self):
        rospy.init_node('make_odom', anonymous=True)
        
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.imuCB)
        rospy.Subscriber("/flag", Bool, self.Flag_CB)
        self.odom_pub = rospy.Publisher('odom',Odometry, queue_size=1)

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.is_gps = False
        self.is_imu = False

        self.flag=Bool()
        self.flag.data=True

        # self.x_offset = 434.32594799867366
        # self.y_offset = -100.06033325288445
        self.x_offset = 334209.0789507285
        self.y_offset = 4143042.501430065
        
        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.prev_x = 0.0  # 이전 x 좌표 값
        self.prev_y = 0.0  # 이전 y 좌표 값

        self.alpha = 0.15  # MAF의 가중치 (0.0에서 1.0 사이의 값)

        self.tf_broadcaster = tf.TransformBroadcaster()

        rate = rospy.Rate(30)

        while self.flag.data:
            if self.is_gps and self.is_imu:
                self.updateOdom()
                rate.sleep()

    def Flag_CB(self,data):
        self.flag = data

    def updateOdom(self):
        # 현재 좌표 계산
        position_x = self.xy_zone[0] - self.x_offset
        position_y = self.xy_zone[1] - self.y_offset

        # MAF를 사용하여 이전 좌표와 현재 좌표의 중간값 계산
        smoothed_x = self.prev_x + self.alpha * (position_x - self.prev_x)
        smoothed_y = self.prev_y + self.alpha * (position_y - self.prev_y)

        self.makeOdomMsg(smoothed_x, smoothed_y)
        self.makeOdomTF(smoothed_x, smoothed_y)

        self.prev_x = smoothed_x
        self.prev_y = smoothed_y

    def makeOdomTF(self, position_x, position_y):
        self.tf_broadcaster.sendTransform(
            (position_x, position_y, 0),
            tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2]),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

    def makeOdomMsg(self, position_x, position_y):
        quaternion = tf.transformations.quaternion_from_euler(self.euler_data[0], self.euler_data[1], self.euler_data[2])
        self.odom.pose.pose.position.x = position_x
        self.odom.pose.pose.position.y = position_y
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = quaternion[0]
        self.odom.pose.pose.orientation.y = quaternion[1]
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]
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