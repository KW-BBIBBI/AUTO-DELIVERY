#!/usr/bin/env python
import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import NavSatFix

class ROSNode:
    def __init__(self):
        rospy.init_node("gps_converter")
        rospy.loginfo("Starting ROSNode as gps_converter")
        rospy.Subscriber("/filtered", GPSMessage, self.gpsCB)
        self.nav_gps_pub = rospy.Publisher("gps", NavSatFix, queue_size=10)
        self.converted_GPS = NavSatFix()

    def gpsCB(self, data):
        # Convert GPSMessage to NavSatFix
        self.converted_GPS.header = data.header
        self.converted_GPS.latitude = data.latitude
        self.converted_GPS.longitude = data.longitude
        self.converted_GPS.altitude = data.altitude
        self.converted_GPS.status = data.status

        # Publish the converted data
        self.nav_gps_pub.publish(self.converted_GPS)

if __name__ == "__main__":
    name_node = ROSNode()
    rospy.spin()