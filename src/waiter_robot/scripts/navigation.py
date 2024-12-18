#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import sys

def send_goal(goal_pos):
    # Inizializza un nodo
    #rospy.init_node("send_goal_node", anonymous=True)

    # Recupera la posizione del tavolo dal server dei parametri
    position = rospy.get_param(goal_pos, None)

    # Crea un client per il server "move_base"
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # Definisci un goal per move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = position['x']
    goal.target_pose.pose.position.y = position['y']
    goal.target_pose.pose.orientation.w = position['z']

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
