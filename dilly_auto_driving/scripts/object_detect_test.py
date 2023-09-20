#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg  import GPSMessage, CollisionData, ObjectStatusList
from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from dilly_auto_driving.utils import findLocalPath, purePursuit, cruiseControl, vaildObject, velocityPlanning, latticePlanner
import os

class object_test:
    def __init__(self):
        rospy.init_node('object_test', anonymous=True)

        rospy.Subscriber("/CollisionData", CollisionData, self.collisiondataCB)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber
        
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0
        self.object_num = 0

        self.vo=vaildObject() ## 장애물 유무 확인 (TrafficLight)

        #def
        self.is_status = False
        self.is_imu = False
        self.is_gps = False
        self.is_obj = False ## 장애물 상태 점검

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)

            self.print_info()
            rate.sleep()  

    def collisiondataCB(self, data):
        self.offset_x = data.global_offset_x
        self.offset_y = data.global_offset_y
        self.offset_z = data.global_offset_z

    def objectInfoCB(self,data):
        self.object_num=data.num_of_npcs+data.num_of_obstacle+data.num_of_pedestrian
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        for num in range(data.num_of_obstacle) :
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian) :
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity.x)

        self.object_info_msg=[object_type,object_pose_x,object_pose_y,object_velocity]
        self.vo.get_object(self.object_num,self.object_info_msg[0],self.object_info_msg[1],self.object_info_msg[2],self.object_info_msg[3])
        self.is_obj=True

    def print_info(self):
        print("status :", self.is_status , "imu :", self.is_imu ,"gps :", self.is_gps ,"obj : ", self.is_obj)
        os.system('clear')
        # print('--------------------status-------------------------')
        # print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        # print('velocity :{} km/h'.format(self.status_msg.velocity.x))
        # print('heading :{} deg'.format(self.status_msg.heading))

        print('--------------------object-------------------------')
        print('object num :{}'.format(self.object_num))
        for i in range(0,self.object_num) :
            print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,self.object_info_msg[0][i],self.object_info_msg[1][i],self.object_info_msg[2][i],self.object_info_msg[3][i]))

        # print('--------------------localization-------------------------')
        # print('all waypoint size: {} '.format(len(self.global_path.poses)))
        # print('current waypoint : {} '.format(self.current_waypoint))

if __name__ == '__main__':
    try:
        test_object = object_test()
    except rospy.ROSInterruptException:
        pass
