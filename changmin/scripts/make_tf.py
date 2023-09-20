#! /usr/bin/env python3

import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import pyproj
import tf
from math import cos,sin,sqrt,pow,atan2,pi


class dilly_status:
    def __init__(self):
        self.heading = 0.0
        self.position = Vector3()
        self.velocity = Vector3()

class GpsConvert:
    def __init__(self):
        rospy.init_node('maketf', anonymous=True)
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.imuCB)
        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)# GPS를 UTM 좌표계로 바꿔주는 코드
        self.status_msg = dilly_status()
        self.x_offset = 334224.240
        self.y_offset = 4143046.829
        self.xy_zone = (0,0)
        self.euler_data = (0,0,0)

        map_status=rospy.Publisher("map_status", bool , queue_size=10)

        rate = rospy.Rate(30) # 50hz
        while not rospy.is_shutdown():
            self.makeTF()
            rate.sleep()
            map_status.publish(self.mapflag)
            
        
    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)

        if data.latitude > 37.419:
            self.mapflag = False    # indoor: False #outdoor: True
        else: self.mapflag = True
        

        
    
    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    
    def makeTF(self):
        self.status_msg.position.x = self.xy_zone[0] - self.x_offset
        self.status_msg.position.y = self.xy_zone[1] - self.y_offset
        self.status_msg.heading = self.euler_data[2] * 180 / pi 
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")


def main():
    
    gps = GpsConvert()
    # try:
    #     rate = rospy.Rate(10) # 50hz
    #     while not rospy.is_shutdown():
    #         gps.makeTF()
    #         rate.sleep()

    # except rospy.ROSInterruptException:
    #     pass

if __name__ == '__main__':
    main()