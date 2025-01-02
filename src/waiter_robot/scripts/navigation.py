#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import sys
from geometry_msgs.msg import Point

def send_goal(goal_pos):
    # Recupera la posizione del tavolo dal server dei parametri
    position = rospy.get_param(goal_pos, None)

    # Crea un client per il server "move_base"
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # Definisci un goal per move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(position['x'],position['y'],0.05)
    goal.target_pose.pose.orientation.x=position['ox']
    goal.target_pose.pose.orientation.y=position['oy']
    goal.target_pose.pose.orientation.z=position['oz']
    goal.target_pose.pose.orientation.w=position['ow']
    # Invia il goal e aspetta che venga raggiunto
    #rospy.loginfo(f"Inviando goal: ({goal_pos})")
    client.send_goal(goal)
    return client.wait_for_result()

    if client.get_result():
        rospy.loginfo("Goal raggiunto con successo!")
    else:
        rospy.logwarn("Il robot non è riuscito a raggiungere il goal.")
    



if __name__ == "__main__":
    try:
        # Esempio di goal da inviare
        send_goal('kitchen')
    except rospy.ROSInterruptException:
        pass
