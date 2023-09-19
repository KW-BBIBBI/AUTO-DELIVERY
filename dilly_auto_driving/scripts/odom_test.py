#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import math

class IMUDataProcessor:
    def __init__(self):
        rospy.init_node('imu_data_processor')
        
        # 초기 변수 설정
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # IMU 데이터를 받기 위한 Subscriber 설정
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imuCB)

    def imuCB(self, imu_msg):
        # IMU 데이터로부터 선형 가속도 (x, y, z) 및 자세 정보 (롤, 피치, 요) 추출
        linear_acceleration = imu_msg.linear_acceleration
        orientation = imu_msg.orientation
        
        # 위치 정보 (가속도를 이용한 두 번째 적분)
        self.x += linear_acceleration.x
        self.y += linear_acceleration.y
        self.z += linear_acceleration.z
        
        # 자세 정보 (쿼터니언을 이용한 오일러 각도 계산)
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        euler = self.quaternion_to_euler(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        # 결과 출력
        rospy.loginfo(f"Position (x, y, z): ({self.x}, {self.y}, {self.z})")
        rospy.loginfo(f"Orientation (Roll, Pitch, Yaw): ({math.degrees(self.roll)}, {math.degrees(self.pitch)}, {math.degrees(self.yaw)})")

    def quaternion_to_euler(self, quaternion):
        # 쿼터니언을 오일러 각도로 변환
        x, y, z, w = quaternion
        
        # 롤, 피치, 요 계산
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

if __name__ == '__main__':
    try:
        imu_data_processor = IMUDataProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
