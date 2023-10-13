#! /usr/bin/python3

import rospy
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

rospy.init_node('application', anonymous=True)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 5.82
goal.target_pose.pose.position.y = 0.786
goal.target_pose.pose.orientation.z = 0.00342
client.send_goal(goal)
wait = client.wait_for_result()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 1.01
goal.target_pose.pose.position.y = 27.4
goal.target_pose.pose.orientation.z = 0.744
client.send_goal(goal)
wait = client.wait_for_result()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id="map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = -50.5
goal.target_pose.pose.position.y = 33.6
goal.target_pose.pose.orientation.z = 0.988
client.send_goal(goal)
wait = client.wait_for_result()
    
os.system("gnome-terminal -- bash -c 'rosservice call /WoowaDillyEventCmd \"request: \n isPickup: False \n deliveryItemIndex: 5\"'")