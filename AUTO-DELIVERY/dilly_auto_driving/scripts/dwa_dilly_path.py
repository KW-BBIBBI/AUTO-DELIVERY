#! /usr/bin/env python3

import rospy
import pyproj
import tf

from math import pi

from geometry_msgs.msg import Vector3
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu

class dilly_status:
    def __init__(self):
        self.heading = 0.0
        self.position = Vector3()

class DWAFollowing:
    def __init__(self):
        rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        rospy.Subscriber('/imu', Imu, self.imuCB)

        self.status_msg = dilly_status()

        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)# GPS를 UTM 좌표계로 바꿔주는 코드
        
        self.x_offset = 0
        self.y_offset = 0
        
        self.xy_zone = (0.0, 0.0)
        self.euler_data = (0.0, 0.0, 0.0)

        self.is_gps = False
        self.is_imu = False
        self.is_status = False
        
    def getDillyStatus(self): ## Vehicle Status Subscriber (Dilly의 위치 값을 저장)
        self.status_msg.position.x = self.xy_zone[0] - self.x_offset
        self.status_msg.position.y = self.xy_zone[1] - self.y_offset
        self.status_msg.heading = self.euler_data[2] * 180 / pi # YAW 값
        # self.status_msg.velocity.x = self.velocity
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "base_link")
        self.is_status=True

    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        self.is_gps = True

    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.is_imu = True

def main():
    rospy.init_node('dwa_following')
    dwa = DWAFollowing()
    try:
        rate = rospy.Rate(50) # 50hz
        while not rospy.is_shutdown():
            dwa.getDillyStatus()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()