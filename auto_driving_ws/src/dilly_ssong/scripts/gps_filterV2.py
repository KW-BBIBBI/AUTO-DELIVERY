#!/usr/bin/env python
import rospy
import numpy as np
from morai_msgs.msg import GPSMessage
from filterpy.kalman import KalmanFilter
from std_msgs.msg import Bool

class GPS_Filter_Node:
    def __init__(self):
        rospy.init_node("GPS_Filtering_Node")
        rospy.loginfo("Starting GPS_Filter_Node as GPS_Filtering_Node")
        rospy.Subscriber("/flag", Bool, self.Flag_CB)
        rospy.Subscriber("/gps_origin", GPSMessage, self.MORAI_GPS_CB)
        self.filtered_pub = rospy.Publisher("gps", GPSMessage, queue_size=10)
        
        self.flag=Bool()
        self.flag.data=True
        # 초기 위치 정보 설정
        self.initial_latitude = 37.416536874388096  # 초기 위도
        self.initial_longitude = 127.13135627199034  # 초기 경도
        self.initial_altitude = 0.6509203910827637  # 초기 고도

        self.Filtered_GPS = GPSMessage()

        # Create a Kalman Filter
        self.kf = KalmanFilter(dim_x=6, dim_z=6)

        # Initialize the state and covariance matrix
        self.kf.x = np.array([self.initial_latitude, self.initial_longitude, self.initial_altitude, 0.0, 0.0, 0.0], dtype=float)
        self.kf.P = np.eye(6)

        # Set the transition matrix
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                             [0, 1, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 1],
                             [0, 0, 0, 1, 0, 0],
                             [0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 1]])

        # Set the process noise covariance matrix
        self.kf.Q = np.array([[1.0e-3, 0, 0, 0, 0, 0],
                             [0, 1.0e-3, 0, 0, 0, 0],
                             [0, 0, 1.0e-3, 0, 0, 0],
                             [0, 0, 0, 1.0e-3, 0, 0],
                             [0, 0, 0, 0, 1.0e-3, 0],
                             [0, 0, 0, 0, 0, 1.0e-3]])

        # Set the observation matrix
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                             [0, 1, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 1, 0, 0],
                             [0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 1]])

        # Set the observation noise covariance matrix
        self.kf.R = np.array([[1.0e-2, 0, 0, 0, 0, 0],
                             [0, 1.0e-2, 0, 0, 0, 0],
                             [0, 0, 1.0e-2, 0, 0, 0],
                             [0, 0, 0, 1.0e-2, 0, 0],
                             [0, 0, 0, 0, 1.0e-2, 0],
                             [0, 0, 0, 0, 0, 1.0e-2]])

        self.queue_size = 20
        self.gps_queue = []

        self.alpha = 1  # Smoothing factor (adjust as needed)
        self.filtered_state = self.kf.x.copy()

        rate = rospy.Rate(50)

        while self.flag.data:
            self.publish_Filtered_GPS()
            rate.sleep()

    def Flag_CB(self,data):
        self.flag = data

    def MORAI_GPS_CB(self, data):
        # Convert GPS data to a measurement vector
        measurement = np.array([data.latitude, data.longitude, data.altitude, 0.0, 0.0, 0.0], dtype=float)

        # Predict the next state
        self.kf.predict()

        # Update the state using the measurement
        self.kf.update(measurement)

        # Apply smoothing to the filtered state
        self.filtered_state = self.alpha * self.kf.x + (1 - self.alpha) * self.filtered_state

        # Update the Filtered_GPS message
        self.Filtered_GPS.latitude = self.filtered_state[0]
        self.Filtered_GPS.longitude = self.filtered_state[1]
        self.Filtered_GPS.altitude = self.filtered_state[2]

    def publish_Filtered_GPS(self):
        self.filtered_pub.publish(self.Filtered_GPS)

def main():
    try:
        gps_pub = GPS_Filter_Node()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
