#! /usr/bin/python3

import rospy
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from morai_msgs.msg import WoowaDillyStatus

rospy.init_node('application', anonymous=True)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 9.18
goal.target_pose.pose.position.y = 15.87
goal.target_pose.pose.orientation.z = 0.05383
client.send_goal(goal)
wait = client.wait_for_result()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 11.6
goal.target_pose.pose.position.y = 17.8
goal.target_pose.pose.orientation.z = 0.861
client.send_goal(goal)
wait = client.wait_for_result()

# 사용할 명령어
os.system("gnome-terminal -- bash -c 'rosservice call /WoowaDillyEventCmd \"request: \n isPickup: True \n deliveryItemIndex: 5\"'")

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 9.18
goal.target_pose.pose.position.y = 5.87
goal.target_pose.pose.orientation.z = 0.00383
client.send_goal(goal)
wait = client.wait_for_result()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 1.38
goal.target_pose.pose.position.y = -6.33
goal.target_pose.pose.orientation.z = 0.00783
client.send_goal(goal)
wait = client.wait_for_result()
    
