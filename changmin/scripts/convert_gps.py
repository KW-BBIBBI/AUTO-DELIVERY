#! /usr/bin/env python3

import rospy
from morai_msgs.msg import GPSMessage
from geographic_msgs.msg import GeoPointStamped

class GpsConvert:
    def __init__(self):
        rospy.Subscriber('/gps', GPSMessage, self.morai_gps_CB)
        self.gps_pub_ = rospy.Publisher( 'gps/geopoint', GeoPointStamped, queue_size=10 )
        self.slam_gps = GeoPointStamped()
        self.slam_gps.header.frame_id = 'gps_'

    def morai_gps_CB(self, data):
        self.slam_gps.position.latitude = data.latitude
        self.slam_gps.position.longitude = data.longitude
        self.slam_gps.position.altitude = data.altitude
    
    def publishGPS(self):
        self.slam_gps.header.stamp = rospy.Time.now()
        self.gps_pub_.publish(self.slam_gps)


def main():
    rospy.init_node('gps_converter')
    gps = GpsConvert()
    try:
        rate = rospy.Rate(50) # 50hz
        while not rospy.is_shutdown():
            gps.publishGPS()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()